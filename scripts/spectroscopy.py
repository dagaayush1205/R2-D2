import cv2
import numpy as np
import rerun as rr
import rerun.blueprint as rrb
from scipy.signal import find_peaks
from serial import Serial
import sys

# --- Configuration ---
ELEMENTS = {
    'Fe3+': [480, 530], 'Fe2+': [510, 560], 'Mn2+': [520, 580],
    'Cu2+': [600, 700], 'Ni2+': [395, 430], 'Cr3+': [570, 620],
    'Co2+': [490, 510], 'Pb2+': [510],      'MnO4-': [525],
    'Cr2O7 2-': [450, 500], 'Ti3+': [500, 600], 'CuCl4 2-': [600, 700],
    'AsO4 3-': [500, 600],  'NO2-': [450, 500], 'I-': [550, 590],
    'SO4 2-': [500, 600]
}

def setup_rerun():
    """Defines the layout using Rerun Blueprints"""
    blueprint = rrb.Blueprint(
        rrb.Horizontal(
            # Left Column: Spectroscopy Data
            rrb.Vertical(
                rrb.Spatial2DView(
                    origin="spectrum",
                    name="Spectroscopy Graph (Wavelength vs Intensity)",
                    visual_bounds=rrb.VisualBounds2D(x_range=[300, 800], y_range=[0, 255])
                ),
                rrb.TextDocumentView(
                    origin="analysis/status",
                    name="Chemical Analysis"
                ),
            ),
            # Right Column: Sensor Time Series
            rrb.Vertical(
                rrb.TimeSeriesView(origin="graph/air_quality", name="Air Quality (MQ/VOC/NO2)"),
                rrb.TimeSeriesView(origin="graph/env", name="Environment (Temp/Hum/Soil)"),
            ),
            column_shares=[2, 1] # Spectroscopy gets 2/3rds screen width
        )
    )
    
    rr.init("BioBox", spawn=True)
    # Note: If connecting to a remote proxy, blueprint support depends on the receiver version.
    # If running locally, this configures the window immediately.
    try:
        rr.send_blueprint(blueprint)
    except Exception as e:
        print(f"Warning: Could not send blueprint (remote viewer might not support it): {e}")

    rr.connect_grpc("rerun+http://127.0.0.1:9876/proxy")

def check_elements(elements, peaks, thresh):
    """
    Formats detection status into a Markdown table for stable reading
    instead of scrolling logs.
    """
    md_output = "## Element Detection Status\n\n"
    md_output += "| Element | Wavelengths (nm) | Status |\n"
    md_output += "| :--- | :--- | :--- |\n"
    
    matches_found = []

    for element, ref_peaks in elements.items():
        match = any(
            any(abs(peak - ref) <= thresh for ref in ref_peaks)
            for peak in peaks
        )
        
        status_icon = "✅ **MATCH**" if match else "..."
        if match:
            matches_found.append(element)
            # Highlight matched rows
            md_output += f"| **{element}** | {ref_peaks} | {status_icon} |\n"
        else:
            md_output += f"| {element} | {ref_peaks} | {status_icon} |\n"

    # Log the full table to a TextDocument (updates in place)
    rr.log("analysis/status", rr.TextDocument(md_output, media_type=rr.MediaType.MARKDOWN))

def log_sensors(ser, frame):
    try:
        if ser.in_waiting > 0:
            line = ser.readline().decode("utf-8", errors='ignore').strip()
            
            # Simple validation
            if not line or line.startswith("mq2"): 
                return

            parts = line.split(",")
            if len(parts) < 7:
                return

            mq2, mq8, voc, no2, soil, temp, hum = map(float, parts)
            
            # Set time once for all metrics in this batch
            rr.set_time("frame_idx", sequence=frame)

            # Log Graphs (Grouped by logical type for the blueprint)
            rr.log("graph/air_quality/mq2", rr.Scalars(mq2))
            rr.log("graph/air_quality/mq8", rr.Scalars(mq8))
            rr.log("graph/air_quality/voc", rr.Scalars(voc))
            rr.log("graph/air_quality/no2", rr.Scalars(no2))
            
            rr.log("graph/env/soil", rr.Scalars(soil))
            rr.log("graph/env/temperature", rr.Scalars(temp))
            rr.log("graph/env/humidity", rr.Scalars(hum))

    except ValueError:
        pass # Ignore partial serial lines
    except Exception as e:
        print(f"Serial Error: {e}")

def log_spectroscope(cap):
    ret, frame = cap.read()
    if not ret:
        print("ERROR: Unable to fetch frame")
        return

    # Process Image
    image = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    # Calculate Spectrum
    pixel_positions = np.arange(image.shape[1])
    calibration_factor = 2.9
    wavelengths = pixel_positions * calibration_factor
    spectrum = np.sum(image, axis=0) # Sum vertical pixels to get intensity per X
    
    # Normalize spectrum for better visualization (Optional: 0-255 range)
    # spectrum = cv2.normalize(spectrum, None, 0, 255, cv2.NORM_MINMAX)

    peaks, _ = find_peaks(spectrum, height=10, threshold=5, distance=5)
    detected_wavelengths = wavelengths[peaks]
    detected_intensities = spectrum[peaks]

    # --- Rerun Logging ---
    
    # 1. Log the Curve (The Graph)
    # We strip the Z dimension to make it purely 2D
    points = np.column_stack((wavelengths, spectrum))
    rr.log(
        "spectrum/curve",
        rr.LineStrips2D(
            [points], 
            colors=[[0, 255, 0]] # Green line
        )
    )

    # 2. Log the Peaks (Red dots on top of the graph)
    if len(detected_wavelengths) > 0:
        peak_points = np.column_stack((detected_wavelengths, detected_intensities))
        rr.log(
            "spectrum/peaks",
            rr.Points2D(
                peak_points, 
                radii=4, 
                colors=[[255, 0, 0]] # Red dots
            )
        )
    else:
        # Clear peaks if none found
        rr.log("spectrum/peaks", rr.Clear(recursive=False))

    # 3. Analyze matches
    check_elements(ELEMENTS, detected_wavelengths, thresh=15)

def main():
    if len(sys.argv) < 3:
        print("Usage: python biobox.py <SERIAL_PORT> <CAMERA_INDEX>")
        print("Example: python biobox.py /dev/ttyUSB0 0")
        return

    PORT = sys.argv[1]
    CAP_SRC = sys.argv[2]
    
    BAUD = 9600
    try:
        ser = Serial(PORT, BAUD, timeout=0.1) # Lower timeout for loop speed
    except Exception as e:
        print(f"Error opening serial: {e}")
        return

    # Accept either an integer camera index or a URL (e.g. MJPEG stream)
    try:
        cap = cv2.VideoCapture(int(CAP_SRC))
    except ValueError:
        cap = cv2.VideoCapture(CAP_SRC)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    if not cap.isOpened():
        print("ERROR: Unable to configure camera")
        return

    # Initialize Rerun and Blueprint
    setup_rerun()

    frame_idx = 0

    try:
        while True:
            # Update Rerun time index
            rr.set_time("frame_idx", sequence=frame_idx)
            
            log_spectroscope(cap)
            log_sensors(ser, frame_idx)
            
            frame_idx += 1
            
            # Optional: Add small sleep if CPU usage is too high
            # time.sleep(0.01) 

    except KeyboardInterrupt:
        print("Exiting...")
    finally:
        cap.release()
        ser.close()

if __name__ == "__main__":
    main()
