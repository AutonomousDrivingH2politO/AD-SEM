#!/usr/bin/env python3
from pytrinamic.connections import ConnectionManager
from pytrinamic.modules import TMCM1260
from time import time

start_deviation_error = 0

def unwrap_position_signed(current_position, previous_position, max_position=2**32):
    """
    Funzione di unwrapping per ottenere una posizione "continua" evitando il roll-over.
    """
    if current_position >= max_position // 2:
        current_position_signed = current_position - max_position
    else:
        current_position_signed = current_position

    if previous_position >= max_position // 2:
        previous_position_signed = previous_position - max_position
    else:
        previous_position_signed = previous_position

    diff = current_position_signed - previous_position_signed
    if diff > max_position // 2:
        return previous_position_signed - (max_position - diff)
    elif diff < -max_position // 2:
        return previous_position_signed + (max_position + diff)
    else:
        return current_position_signed

class Stepper:
    def __init__(self, interface, data_rate, module_id, max_velocity, max_acc, MaxDeceleration, V1, A1, D1):
        global start_deviation_error
        if interface == "can":
            communication = "socketcan_tmcl"
        elif interface == "usb":
            communication = "usb_tmcl"
        else:
            raise ValueError(f"Cannot use {interface} communication, possible values are 'can' or 'usb'")

        # Inizializzazione dell'interfaccia del motore:
        self.interface = ConnectionManager(f"--interface {communication} --port can1 --data-rate {data_rate}").connect()
        self.module_id = module_id
        self.data_rate = data_rate
        self.module = TMCM1260(self.interface, module_id=module_id)
        self.motor = self.module.motors[0]

        # Impostazione dei parametri del motore:
        self.motor.set_axis_parameter(self.motor.AP.MaxVelocity, max_velocity)
        self.motor.set_axis_parameter(self.motor.AP.MaxAcceleration, max_acc)
        self.motor.set_axis_parameter(self.motor.AP.MaxDeceleration, MaxDeceleration)
        self.motor.set_axis_parameter(self.motor.AP.V1, V1)
        self.motor.set_axis_parameter(self.motor.AP.A1, A1)
        self.motor.set_axis_parameter(self.motor.AP.D1, D1)
        self.motor.set_axis_parameter(self.motor.AP.StartVelocity, 1_000)
        self.motor.set_axis_parameter(self.motor.AP.StopVelocity, 1_000)
        self.motor.set_axis_parameter(self.motor.AP.RampWaitTime, 0)
        self.motor.set_axis_parameter(self.motor.AP.MaxCurrent, 200)
        self.motor.set_axis_parameter(self.motor.AP.StandbyCurrent, 100)
        self.motor.set_axis_parameter(self.motor.AP.SG2Threshold, 11)
        self.motor.set_axis_parameter(self.motor.AP.SG2FilterEnable, 0)
        self.motor.set_axis_parameter(self.motor.AP.SmartEnergyStallVelocity, 0)
        self.motor.set_axis_parameter(self.motor.AP.SmartEnergyHysteresis, 15)
        self.motor.set_axis_parameter(self.motor.AP.SmartEnergyHysteresisStart, 0)
        self.motor.set_axis_parameter(self.motor.AP.SECUS, 1)
        self.motor.set_axis_parameter(self.motor.AP.SmartEnergyThresholdSpeed, 7_999_774)
        self.motor.drive_settings.boost_current = 0
        self.motor.drive_settings.microstep_resolution = self.motor.ENUM.MicrostepResolution256Microsteps

        # Impostazione della posizione iniziale: viene azzerata l'encoder a ogni istanziazione
        self.virtual_position = self.motor.get_actual_position()
        self.motor.set_axis_parameter(self.motor.AP.EncoderPosition, 0)
        self.effective_position = self.motor.get_axis_parameter(self.motor.AP.EncoderPosition)
        start_deviation_error = self.effective_position - self.virtual_position
        print(f"{self.virtual_position}")
        # Inizializza le variabili per l'unwrapping
        self.last_encoder_position = self.motor.get_axis_parameter(self.motor.AP.EncoderPosition)
        self.unwrapped_position = 0

        print(self.motor.drive_settings)

    def update_unwrapped_position(self):
        """
        Aggiorna e restituisce la posizione "unwrapped" basata sulla lettura corrente dell'encoder.
        """
        current_encoder = self.motor.get_axis_parameter(self.motor.AP.EncoderPosition)
        self.unwrapped_position = unwrap_position_signed(current_encoder, self.last_encoder_position)
        self.last_encoder_position = current_encoder
        return self.unwrapped_position

    def computeErr(self):
        global start_deviation_error
        t = 200
        deviation_error = self.effective_position - self.virtual_position - start_deviation_error

        if abs(deviation_error) > t:
            return True, deviation_error
        else:
            return False, 0

    def brake(self):
        """
        Sequenza di frenata progressiva senza sleep:
        1. Ritrae il motore di 70.000 tick
        2. Avanza lentamente correggendo eventuale slittamento
        3. Allinea AP con encoder
        """
        start_time = time()
        runtime = 0
        min_pos_rel = -10
        print("[Stepper] Braking...")

        # Step 1: ritorno rapido
        self.start_pos = self.motor.get_axis_parameter(self.motor.AP.EncoderPosition)
        self.end_pos = self.start_pos - 70000
        self.move_to(self.end_pos)
        print(f"[Stepper] Initial encoder position: {self.start_pos}")

        while not self.motor.get_position_reached():
            pass  # attesa attiva

        # Step 2: avanzamento incrementale con correzione slip
        self.devErr = 0
        loop_start = time()

        while runtime < 10:
            pos_ap = self.motor.get_axis_parameter(self.motor.AP.ActualPosition)
            pos_enc = self.motor.get_axis_parameter(self.motor.AP.EncoderPosition)
            print(f"[Stepper] Actual: {pos_ap} | Encoder: {pos_enc}")

            slipped, self.devErr = self.compute_slip_error()
            if slipped:
                print(f"[WARN] slip detected → correcting by {self.devErr} ticks")
                self.move_by(self.devErr)
                while not self.motor.get_position_reached():
                    pass

            self.move_by(min_pos_rel)
            while not self.motor.get_position_reached():
                pass

            runtime = time() - loop_start

        # Step 3: ritorno alla posizione bilanciata
        print("[Stepper] Returning to aligned position")
        pos_ap = self.motor.get_axis_parameter(self.motor.AP.ActualPosition)
        pos_enc = self.motor.get_axis_parameter(self.motor.AP.EncoderPosition)
        diff = pos_ap - pos_enc

        if abs(diff) > 0:
            self.move_to(pos_ap)
            while not self.motor.get_position_reached():
                pass

        print("[Stepper] Brake sequence completed")

    def move_stepper(self, step):
        """
        Esegue il comando di spostamento sullo stepper.
        """
        print(f"Moving stepper to command: {step}")
        #self.motor.set_axis_parameter(self.motor.AP.SmartEnergyStallVelocity, 0)
        self.motor.move_to(step)

    def disconnect_motor(self):
        self.motor.stop()
        self.interface.close()