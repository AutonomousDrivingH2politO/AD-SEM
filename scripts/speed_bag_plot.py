import matplotlib.pyplot as plt
from pathlib import Path
from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore

# Configuration
bagpath = Path('rosbag2_2026_06_03-14_21_34')
ENCODER_DEADBAND = 2.016  # The speed at which encoder data becomes garbage

timestamps = []
speeds = []
stop_command_times = []
previous_stop_state = False
start_time = None

print(f"Processing bag for visualization: {bagpath}...")

# Create a typestore containing standard ROS 2 message definitions
typestore = get_typestore(Stores.LATEST)

with AnyReader([bagpath], default_typestore=typestore) as reader:
    topics_of_interest = ['/ECU/speed', '/commands/stop']
    connections = [x for x in reader.connections if x.topic in topics_of_interest]

    for connection, timestamp, rawdata in reader.messages(connections=connections):
        msg = reader.deserialize(rawdata, connection.msgtype)
        time_sec = timestamp / 1e9  # Convert nanoseconds to seconds
        
        if start_time is None:
            start_time = time_sec
            
        relative_time = time_sec - start_time

        if connection.topic == '/ECU/speed':
            speed_val = getattr(msg, 'data', getattr(msg, 'speed', 0.0))
            timestamps.append(relative_time)
            speeds.append(speed_val)

        elif connection.topic == '/commands/stop':
            is_stop = getattr(msg, 'data', False)
            if is_stop and not previous_stop_state:
                stop_command_times.append(relative_time)
            previous_stop_state = is_stop

# Plotting the data
plt.figure(figsize=(12, 6))
plt.plot(timestamps, speeds, label='Raw /ECU/speed (km/h)', color='blue', linewidth=1.5, alpha=0.5)

# Calculate and plot the macro deceleration for each stop event
if stop_command_times:
    for i, stop_time in enumerate(stop_command_times):
        engage_time = stop_time + 2.0
        
        try:
            # Step 1: Find start of braking (t + 2.0s)
            idx_start = next(idx for idx, t in enumerate(timestamps) if t >= engage_time)
            
            # Step 2: Find end of braking (when it hits the deadband)
            idx_stop = idx_start
            while idx_stop < len(speeds) and speeds[idx_stop] > ENCODER_DEADBAND:
                idx_stop += 1
            
            if idx_stop < len(speeds):
                t_start = timestamps[idx_start]
                t_end = timestamps[idx_stop]
                
                # Step 3: Calculate Deceleration
                # Convert km/h to m/s. Assume deadband means we reached 0 m/s.
                v_start_ms = speeds[idx_start] / 3.6
                v_end_ms = 0.0 
                dt = t_end - t_start
                
                if dt > 0:
                    decel_ms2 = (v_end_ms - v_start_ms) / dt
                    
                    print(f"--- Stop Event {i+1} ---")
                    print(f"Triggered at: {stop_time:.2f}s | Engaged at: {engage_time:.2f}s")
                    print(f"Braking Duration: {dt:.2f}s")
                    print(f"Initial Speed: {speeds[idx_start]:.2f} km/h ({v_start_ms:.2f} m/s)")
                    print(f"Calculated Deceleration: {decel_ms2:.3f} m/s²")
                    
                    # Step 4: Plot the idealized deceleration slope
                    # We plot it in km/h on the graph so it aligns with the raw data
                    plt.plot([t_start, t_end], [speeds[idx_start], 0], 
                             color='green', linewidth=2.5, 
                             label=f'Calculated Decel' if i == 0 else None)
                    
                    # Step 5: Annotate the plot with the deceleration value
                    x_mid = (t_start + t_end) / 2
                    y_mid = speeds[idx_start] / 2
                    plt.text(x_mid + 0.2, y_mid, f"{decel_ms2:.2f} m/s²", 
                             color='green', fontsize=10, fontweight='bold',
                             bbox=dict(facecolor='white', alpha=0.8, edgecolor='none', pad=2))
                    
        except StopIteration:
            print(f"[!] Could not find matching speed data for stop event at {stop_time:.2f}s")

        # Mark vertical lines
        label_trigger = 'Stop Trigger' if i == 0 else None
        label_engage = 'Brake Engaged (+2s)' if i == 0 else None
        plt.axvline(x=stop_time, color='red', linestyle='--', label=label_trigger)
        plt.axvline(x=engage_time, color='orange', linestyle='-', linewidth=2, label=label_engage)

else:
    print("[?] /commands/stop (True) was not detected.")

# Graph customization
plt.title(f'Vehicle Deceleration Analysis: {bagpath.name}')
plt.xlabel('Time Elapsed (seconds)')
plt.ylabel('Speed (km/h)')
plt.axhline(y=ENCODER_DEADBAND, color='black', linestyle=':', label='Encoder Deadband (2.016)')
plt.grid(True, linestyle=':', alpha=0.6)
plt.legend()

# Save and show
output_plot = f"{bagpath.name}_deceleration_macro.png"
plt.savefig(output_plot, dpi=300)
print(f"Plot successfully saved to {output_plot}")
plt.show()
