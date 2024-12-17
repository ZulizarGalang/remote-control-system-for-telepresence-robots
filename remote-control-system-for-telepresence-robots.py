import numpy as np
import serial
import time
from threading import Thread, Lock
import queue
import pyrebase
import glob
import sys
import os
from datetime import datetime
import pandas as pd
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import StandardScaler
from sklearn.metrics import confusion_matrix, classification_report
import logging

# Set up logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler('robot_controller.log'),
        logging.StreamHandler()
    ]
)

# Firebase configuration
firebase_config = {
    "apiKey": "AIzaSyDhvqWL2ictmZTlJnn-jRzRlJejAHMTsn0",
    "authDomain": "skripsiijul-28eed.firebaseapp.com",
    "databaseURL": "https://skripsiijul-28eed-default-rtdb.asia-southeast1.firebasedatabase.app",
    "storageBucket": "skripsiijul-28eed.appspot.com",
}

class NeuralNetwork:
    def __init__(self, input_size, hidden_size=64, output_size=4):
        self.input_size = input_size
        self.hidden_size = hidden_size
        self.output_size = output_size
        
        # Initialize weights and biases
        np.random.seed(42)  # For reproducibility
        self.weights_input_hidden = np.random.randn(input_size, hidden_size)
        self.bias_hidden = np.zeros(hidden_size)
        self.weights_hidden_output = np.random.randn(hidden_size, output_size)
        self.bias_output = np.zeros(output_size)
        
        self.label_mapping = {'N': 0, 'F': 1, 'L': 2, 'R': 3}
        self.reverse_label_mapping = {v: k for k, v in self.label_mapping.items()}
        
    def relu(self, x):
        return np.maximum(0, x)
        
    def softmax(self, x):
        exp_values = np.exp(x - np.max(x, axis=1, keepdims=True))
        return exp_values / np.sum(exp_values, axis=1, keepdims=True)
    
    def train(self, X_train, y_train, learning_rate=0.2, epochs=50, batch_size=64):
        best_accuracy = 0
        
        for epoch in range(epochs):
            total_loss = 0
            for i in range(0, len(X_train), batch_size):
                batch_X = X_train[i:i+batch_size]
                batch_y = y_train[i:i+batch_size]
                
                # Forward pass
                hidden_layer_input = np.dot(batch_X, self.weights_input_hidden) + self.bias_hidden
                hidden_layer_output = self.relu(hidden_layer_input)
                output_layer_input = np.dot(hidden_layer_output, self.weights_hidden_output) + self.bias_output
                output_layer_output = self.softmax(output_layer_input)
                
                # Backpropagation
                loss = -np.log(output_layer_output[range(len(batch_y)), batch_y]).mean()
                total_loss += loss
                
                d_output = output_layer_output
                d_output[range(len(batch_y)), batch_y] -= 1
                d_output /= len(batch_y)
                
                d_hidden = np.dot(d_output, self.weights_hidden_output.T)
                d_hidden[hidden_layer_output <= 0] = 0
                
                # Update weights and biases
                self.weights_hidden_output -= learning_rate * np.dot(hidden_layer_output.T, d_output)
                self.bias_output -= learning_rate * np.sum(d_output, axis=0)
                self.weights_input_hidden -= learning_rate * np.dot(batch_X.T, d_hidden)
                self.bias_hidden -= learning_rate * np.sum(d_hidden, axis=0)
            
            logging.info(f'Epoch {epoch + 1}, Average Loss: {total_loss / (len(X_train) / batch_size)}')
    
    def predict(self, X):
        hidden_layer_input = np.dot(X, self.weights_input_hidden) + self.bias_hidden
        hidden_layer_output = self.relu(hidden_layer_input)
        output_layer_input = np.dot(hidden_layer_output, self.weights_hidden_output) + self.bias_output
        output_layer_output = self.softmax(output_layer_input)
        
        predictions = np.argmax(output_layer_output, axis=1)
        confidence = np.max(output_layer_output, axis=1) * 100
        
        return predictions, confidence
    
    def save_model(self, filename='model_params.npz'):
        np.savez(filename,
                 weights_input_hidden=self.weights_input_hidden,
                 bias_hidden=self.bias_hidden,
                 weights_hidden_output=self.weights_hidden_output,
                 bias_output=self.bias_output)
        
        # Save scaler parameters separately
        if hasattr(self, 'scaler'):
            np.savez('scaler_params.npz',
                     scale=self.scaler.scale_,
                     mean=self.scaler.mean_)
    
    def load_model(self, filename='model_params.npz'):
        data = np.load(filename)
        self.weights_input_hidden = data['weights_input_hidden']
        self.bias_hidden = data['bias_hidden']
        self.weights_hidden_output = data['weights_hidden_output']
        self.bias_output = data['bias_output']

class RobotController:
    def __init__(self, port, baud_rate=230400):
        self.serial = serial.Serial(port, baud_rate)
        self.neural_net = None
        self.firebase = pyrebase.initialize_app(firebase_config).database()
        self.data_queue = queue.Queue(maxsize=1)
        self.command_lock = Lock()
        
        self.last_prediction = 'N'  # Default to 'N' when no obstacles
        self.last_confidence = 0
        self.last_sensor_data = np.array([1.0, 1.0, 1.0, 1.0, 1.0])  # Default sensor values
        self.prediction_count = 0
        self.start_time = time.time()
        self.last_data_received_time = time.time()
        
        # Add new variables to track last commands
        self.last_movement = None
        self.last_rotation = None
        
        self.running = True
        self.setup_neural_network()
        self.process_thread = Thread(target=self._process_data)
        self.process_thread.start()
    
    def setup_neural_network(self):
        # Load and preprocess training data
        data = pd.read_csv('3newfiksbangetparahultrasonicdataset_ijul.csv')
        X = data.iloc[:, :-1].values
        y = data.iloc[:, -1].values
        
        # Initialize scaler and neural network
        scaler = StandardScaler()
        X_scaled = scaler.fit_transform(X)
        
        # Convert labels
        label_mapping = {'N': 0, 'F': 1, 'L': 2, 'R': 3}
        y_encoded = np.array([label_mapping[label] for label in y])
        
        # Split data
        X_train, X_test, y_train, y_test = train_test_split(
            X_scaled, y_encoded, test_size=0.2, random_state=42)
        
        # Initialize and train neural network
        self.neural_net = NeuralNetwork(input_size=X_scaled.shape[1])
        self.neural_net.scaler = scaler
        
        # Try to load existing model, train if not available
        try:
            self.neural_net.load_model()
            logging.info("Loaded existing model parameters")
        except FileNotFoundError:
            logging.info("Training new model...")
            self.neural_net.train(X_train, y_train)
            self.neural_net.save_model()
            
            # Evaluate model
            predictions, _ = self.neural_net.predict(X_test)
            accuracy = np.mean(predictions == y_test)
            logging.info(f'Test Accuracy: {accuracy}')
            logging.info('\nClassification Report:\n' + 
                        classification_report(y_test, predictions))

    def _read_serial(self):
        while self.running:
            try:
                if self.serial.in_waiting:
                    line = self.serial.readline().decode().strip()
                    try:
                        raw_data = [float(x) for x in line.split(',')]
                        if len(raw_data) == 5:
                            # Reorder the data - switch front right and front left
                            front_center = raw_data[0]
                            front_left = raw_data[1]
                            front_right = raw_data[2]
                            left = raw_data[3]
                            right = raw_data[4]
                            
                            # Create reordered array
                            data = np.array([front_center, front_right, front_left, left, right])
                            self.last_sensor_data = data
                            self.last_data_received_time = time.time()
                            
                            try:
                                self.data_queue.get_nowait()  # Clear old data
                            except queue.Empty:
                                pass
                            self.data_queue.put(data)
                    except ValueError as e:
                        logging.warning(f"Invalid data format received: {line}")
                        continue
                
                # If no new data received for 1 second, assume all sensors read 1.0
                elif time.time() - self.last_data_received_time > 1.0:
                    self.last_sensor_data = np.array([1.0, 1.0, 1.0, 1.0, 1.0])
                    self.last_prediction = 'N'
                    self.last_confidence = 100.0
                    
                    try:
                        self.data_queue.get_nowait()
                    except queue.Empty:
                        pass
                    self.data_queue.put(self.last_sensor_data)
                    self.last_data_received_time = time.time()
                    
            except Exception as e:
                logging.error(f"Serial read error: {e}")
                time.sleep(0.1)

    def _display_status(self):
        os.system('cls' if os.name == 'nt' else 'clear')
        
        elapsed_time = time.time() - self.start_time
        pps = self.prediction_count / elapsed_time if elapsed_time > 0 else 0
        current_time = datetime.now().strftime("%H:%M:%S")
        
        print("=" * 50)
        print(f"Robot Control Status - {current_time}")
        print("=" * 50)
        
        if self.last_sensor_data is not None:
            print("\nSensor Readings:")
            sensor_labels = ['Front Center', 'Front Right', 'Front Left', 'Left', 'Right']
            for i, label in enumerate(sensor_labels):
                print(f"{label:<12}: {self.last_sensor_data[i]:.2f}")
        
        print("\nNeural Network Output:")
        print(f"Prediction   : {self.last_prediction}")
        print(f"Confidence   : {self.last_confidence:.2f}%")
        
        print("\nJoystick Status:")
        print(f"Last Movement: {self.last_movement or 'None'}")
        print(f"Last Rotation: {self.last_rotation or 'None'}")
        
        print(f"\nPerformance:")
        print(f"Predictions/s: {pps:.2f}")
        print(f"Total Predictions: {self.prediction_count}")
        print("=" * 50)

    def _process_data(self):
        last_firebase_update = 0
        last_display_update = 0
        FIREBASE_UPDATE_INTERVAL = 0.1
        DISPLAY_UPDATE_INTERVAL = 0.2

        while self.running:
            try:
                data = self.data_queue.get(timeout=0.1)
                
                # Check if all sensors read 1.0 (no obstacles)
                if np.allclose(data, 1.0, rtol=1e-3):
                    prediction = 'N'
                    confidence = 100.0
                else:
                    # Only run neural network if there are obstacles detected
                    scaled_data = self.neural_net.scaler.transform(data.reshape(1, -1))
                    predictions, confidence = self.neural_net.predict(scaled_data)
                    prediction = self.neural_net.reverse_label_mapping[predictions[0]]
                
                self.last_prediction = prediction
                self.last_confidence = confidence[0] if isinstance(confidence, np.ndarray) else confidence
                self.prediction_count += 1
                
                current_time = time.time()
                
                # Handle control logic
                with self.command_lock:
                    if prediction != 'N':
                        self.serial.write(prediction.encode())
                
                # Update display
                if current_time - last_display_update >= DISPLAY_UPDATE_INTERVAL:
                    self._display_status()
                    last_display_update = current_time
                
                # Update Firebase and handle joystick commands
                if current_time - last_firebase_update >= FIREBASE_UPDATE_INTERVAL:
                    if prediction == 'N':
                        self.firebase.child("Robot").set("N")
                        self.serial.write('S'.encode())
                        
                        # Get both movement and rotation commands
                        command1 = self.firebase.child("Joystick").child("DirectionXY0").get().val()
                        command2 = self.firebase.child("Joystick").child("DirectionXY1").get().val()
                        
                        with self.command_lock:
                            # Handle rotation commands
                            if command2 in ["T", "U", "V", "W", "X", "Y", "J", "Z"] and command2 != self.last_rotation:
                                logging.info(f"New rotation command: {command2}")
                                self.serial.write(command2.encode())
                                self.last_rotation = command2
                            
                            # Handle movement commands
                            if command1 in ["D", "B", "K", "P", "S", "C", "G", "I", "H"] and command1 != self.last_movement:
                                logging.info(f"New movement command: {command1}")
                                self.serial.write(command1.encode())
                                self.last_movement = command1
                    else:
                        self.firebase.child("Robot").remove()
                    last_firebase_update = current_time
                    
            except queue.Empty:
                continue
            except Exception as e:
                logging.error(f"Processing error: {e}")
                time.sleep(0.1)

    def start(self):
        self.serial_thread = Thread(target=self._read_serial)
        self.serial_thread.start()

    def stop(self):
        self.running = False
        self.process_thread.join()
        self.serial_thread.join()
        self.serial.close()

def find_arduino_port():
    if sys.platform.startswith('win'):
        ports = ['COM%s' % (i + 1) for i in range(256)]
    elif sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
        ports = glob.glob('/dev/tty[A-Za-z]*')
    elif sys.platform.startswith('darwin'):
        ports = glob.glob('/dev/tty.*')
    else:
        raise EnvironmentError('Unsupported platform')

    for port in ports:
        try:
            ser = serial.Serial(port, 230400)
            ser.close()
            return port
        except (OSError, serial.SerialException):
            continue
    return None

def main():
    arduino_port = find_arduino_port()
    
    if not arduino_port:
        logging.error("Arduino not found! Please check the connection.")
        return
    
    logging.info(f"Found Arduino on port: {arduino_port}")
    logging.info("Initializing robot controller...")
    
    controller = RobotController(port=arduino_port)
    try:
        controller.start()
        logging.info("Controller started. Press Ctrl+C to stop.")
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        logging.info("\nStopping controller...")
        controller.stop()
        logging.info("Controller stopped.")

if __name__ == "__main__":
    main()