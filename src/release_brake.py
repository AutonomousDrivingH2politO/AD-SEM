from pytrinamic.connections import ConnectionManager
from pytrinamic.modules import TMCM1260

iface = ConnectionManager("--interface socketcan_tmcl --port can1 --data-rate 1000000").connect()
module = TMCM1260(iface, module_id=1)
motor = module.motors[0]

motor.stop()
motor.set_axis_parameter(motor.AP.MaxCurrent, 0)
motor.set_axis_parameter(motor.AP.StandbyCurrent, 0)

iface.close()
