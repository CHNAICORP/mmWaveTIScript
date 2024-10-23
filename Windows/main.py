import re
import serial
import time
import subprocess
import os
import platform
from datetime import datetime, timedelta
import glob
import logging
import sys

def setup_logging():
    log_directory = './logs'  # Directory for log files
    log_name = 'mmwave.log'  # Log file name
    log_filename = os.path.join(log_directory, log_name)  # Full path for the log file

    try:
        # Check if the log directory exists, if not, create it
        if not os.path.exists(log_directory):
            os.makedirs(log_directory)
            print(f"Created log directory at {log_directory}")
        else:
            print(f"Log directory already exists at {log_directory}")
        
        # Set the log file location and log level
        logging.basicConfig(filename=log_filename,
                            level=logging.INFO,
                            format='%(asctime)s - %(levelname)s - %(message)s',
                            datefmt='%Y-%m-%d %H:%M:%S')
        print(f"Logging setup complete. Log file at {log_filename}")

        # Test the logging functionality
        logging.info("Logging setup successful.")
    except Exception as e:
        print(f"Error setting up logging: {e}")
        raise

def run_command(command):
    time.sleep(2)  # Delay to ensure command execution readiness
    logging.info(f'Command executed: {command}')  # Log the executed command
    if platform.system() == "Windows":
        subprocess.run(command)  # Run command on Windows
    else:
        # On Linux, you may need to use other methods to execute, such as calling a shell script or running an executable directly
        subprocess.run(command, shell=True)  # Run command on Linux

def config_dca1000():
    # Determine the configuration file path based on the operating system
    config_file = 'json/cf.json' if platform.system() == 'Linux' else 'json\\cf.json'
    command_prefix = './' if platform.system() == 'Linux' else ''  # Command prefix for Linux
    executable = 'DCA1000EVM_CLI_Control.exe'  # Executable name
    
    # Build command paths for FPGA configuration and recording
    fpga_command = [command_prefix + executable, 'fpga', config_file]
    record_command = [command_prefix + executable, 'record', config_file]
    start_record_command = [command_prefix + executable, 'start_record', config_file]
    query_status_command = f"{executable} query_status {config_file}"
    
    # Execute commands
    run_command(fpga_command)  # Configure FPGA
    run_command(record_command)  # Start recording
    run_command(start_record_command)  # Begin recording process
    run_command(query_status_command)  # Query the status

def config_iwr6843(serial_port, CLIport):
    file_cfg = 'cfg/profile.cfg'  # Configuration file for IWR6843
    # print('Sending ' + file_cfg + ' to IWR6843AOP on ' + serial_port)
    # logging.info(f'Sending {file_cfg} to IWR6843AOP on {serial_port}')
    try:
        # Open the configuration file and read commands
        with open(file_cfg, 'r') as file:
            config = [line.rstrip('\r\n') for line in file]
            for i in config:
                if i == '' or i[0] == '%':  # Skip empty lines and comments
                    continue
                current_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]  # Format time to milliseconds
                logging.info(f'[{current_time}] >>> {i}')  # Log the command being sent
                print(f'[{current_time}] >>> {i}')  # Output command with timestamp
                CLIport.write((i + '\n').encode())  # Send command to the device
                if i == 'sensorStart':  # Stop sending commands after starting the sensor
                    break
                time.sleep(0.1)  # Delay between commands
    except serial.SerialException as e:
        print(f"Serial port error: {e}")  # Print serial port error
        logging.info(f"Serial port error: {e}")  # Log serial port error

def stop_and_reset():
    config_file = 'json/cf.json'  # Configuration file for stopping and resetting
    executable = 'DCA1000EVM_CLI_Control.exe'  # Executable name

    current_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]  # Get current time
    stop_record_command = f"{executable} stop_record {config_file}"  # Command to stop recording
    reset_fpga_command = f"{executable} reset_fpga {config_file}"  # Command to reset FPGA
    reset_ar_device_command = f"{executable} reset_ar_device {config_file}"  # Command to reset AR device

    # Execute stop recording command
    print(f'[{current_time}] Executing stop_record command...')
    logging.info(f'[{current_time}] Executing stop_record command...')
    run_command(stop_record_command)  # Run stop recording command

    # Execute reset FPGA command
    print(f'[{current_time}] Executing reset_fpga command...')
    logging.info(f'[{current_time}] Executing reset_fpga command...')
    run_command(reset_fpga_command)  # Run reset FPGA command

    print(f'[{current_time}] Executing reset_ar_device command...')
    logging.info(f'[{current_time}] Executing reset_ar_device command...')
    run_command(reset_ar_device_command)  # Run reset AR device command

def extract_times_from_log(file_path):
    start_time = ""  # Initialize start time
    end_time = ""  # Initialize end time

    with open(file_path, 'r') as file:  # Open the log file
        for line in file:  # Read each line
            if "Capture start time" in line:  # Check for start time
                start_time = line.split(" - ")[1].strip()  # Extract start time
            elif "Capture end time" in line:  # Check for end time
                end_time = line.split(" - ")[1].strip()  # Extract end time

    return start_time, end_time  # Return extracted times

def format_time_for_filename(time_str):
    # Remove the weekday part, keep only the date and time part, and move the year to after the date
    match = re.match(r'^[A-Za-z]{3} (\w{3}) (\d{2}) (\d{2}:\d{2}:\d{2}\.\d{3}) (\d{4})', time_str)
    if match:
        month_day = f"{match.group(1)}_{match.group(2)}_{match.group(4)}"
        time_part = match.group(3).replace(":", "_").replace(".", "_")
        return f"{month_day}_{time_part}"
    else:
        return time_str.replace(" ", "_").replace(":", "_").replace(".", "_")

def rename_bin_files(data_dir, start_time, end_time):
    try:
        # Find all .bin files in the data_dir directory
        bin_files = glob.glob(os.path.join(data_dir, "*.bin"))
        if not bin_files:
            print("No .bin files found to rename.")
            logging.info("No .bin files found to rename.")
            return
        
        # Format time
        start_time_formatted = format_time_for_filename(start_time)
        end_time_formatted = format_time_for_filename(end_time)

        if len(bin_files) == 1:
            # When there is only one .bin file
            new_file_name = f"{start_time_formatted}-{end_time_formatted}.bin"
            new_file_path = os.path.join(data_dir, new_file_name)
            os.rename(bin_files[0], new_file_path)
            print(f"File renamed from {bin_files[0]} to {new_file_path}")
            logging.info(f"File renamed from {bin_files[0]} to {new_file_path}")
        else:
            # When there are multiple .bin files
            for idx, bin_file in enumerate(bin_files):
                base_name = os.path.basename(bin_file)
                new_file_name = f"{start_time_formatted}_{end_time_formatted}_{idx}.bin"
                new_file_path = os.path.join(data_dir, new_file_name)
                os.rename(bin_file, new_file_path)
                print(f"File renamed from {bin_file} to {new_file_path}")
                logging.info(f"File renamed from {bin_file} to {new_file_path}")
    except Exception as e:
        print(f"Error renaming file: {e}")
        logging.info(f"Error renaming file: {e}")

def find_log_file(data_dir):
    log_files = glob.glob(os.path.join(data_dir, "adc_data_Raw_LogFile.csv"))
    if log_files:
        return log_files[0]
    else:
        return None

def display_progress(duration):
    start_time = time.time()
    while True:
        elapsed_time = time.time() - start_time
        progress = elapsed_time / duration
        bar_length = 50
        block = int(round(bar_length * progress))
        text = f"\rProgress: [{'#' * block + '-' * (bar_length - block)}] {int(progress * 100)}%"
        sys.stdout.write(text)
        sys.stdout.flush()
        if elapsed_time >= duration:
            break
        time.sleep(1)
    print()  # Move to the next line after the progress bar is complete

def format_timedelta(td):
    seconds = td.total_seconds()
    milliseconds = int((seconds - int(seconds)) * 1000)
    seconds = int(seconds)
    minutes, seconds = divmod(seconds, 60)
    hours, minutes = divmod(minutes, 60)
    days, hours = divmod(hours, 24)
    return f"{days:02}:{hours:02}:{minutes:02}:{seconds:02}.{milliseconds:03}"

def main():
    setup_logging()  # Set up logging
    data_dir = './data'  # Directory for data files
    serial_port_CLI = '/dev/ttyACM0' if platform.system() == 'Linux' else 'COM4'  # Serial port based on OS
    start_time = None  # Initialize start time
    end_time = None  # Initialize end time

    try:
        with serial.Serial(serial_port_CLI, 115200, timeout=1) as CLIport:  # Open serial port
            sending = False  # Flag to indicate if data is being sent
    
            while True:
                if sending:
                    print("Program started. Enter 'q' to quit, 's' to stop collecting data, 'r' to reset mmWare Ladar.")
                    logging.info("Program started. Enter 'q' to quit, 's' to stop collecting data, 'r' to reset mmWare Ladar.")
                else:
                    print("Program started. Enter 'q' to quit, 's' to start automatically collecting data, 'r' to reset mmWare Ladar.")
                    logging.info("Program started. Enter 'q' to quit, 's' to start automatically collecting data, 'r' to reset mmWare Ladar.")

                user_input = input("Enter your command: ")  # Get user input

                if user_input.lower() == 'q':  # Quit command
                    if sending:
                        stop_and_reset()  # Stop and reset if sending
                    break

                elif user_input.lower() == 's':  # Start/stop command
                    if sending:
                        log_file_path = find_log_file(data_dir)  # Find log file

                        if log_file_path:
                            start_time, end_time = extract_times_from_log(log_file_path)  # Extract times from log

                            if start_time and end_time:
                                rename_bin_files(data_dir, start_time, end_time)  # Rename .bin files
                                stop_and_reset()  # Stop and reset
                                print("Data collection completed!")  # Notify completion
                                logging.info("Data collection completed!")  # Log completion
                            else:
                                print("Start time or end time not found in log file.")  # Notify missing times
                                logging.info("Start time or end time not found in log file.")  # Log missing times
                        else:
                            print("Log file adc_data_Raw_LogFile.csv not found in the directory.")  # Notify missing log file
                            logging.info("Log file adc_data_Raw_LogFile.csv not found in the directory.")  # Log missing log file
                        sending = False  # Reset sending flag
                        
                    else:
                        config_dca1000()  # Configure DCA1000
                        config_iwr6843(serial_port_CLI, CLIport)  # Configure IWR6843
                        sending = True  # Set sending flag to True
                
                elif user_input.lower() == 'r':  # Reset command
                    stop_and_reset()  # Stop and reset
                    print("Reset mmWare Ladar finish!")  # Notify reset completion
                    logging.info("Reset mmWare Ladar finish!")  # Log reset completion

    except serial.SerialException as e:
        print(f"Serial port error: {e}")
        logging.info(f"Serial port error: {e}")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
        logging.info(f"An unexpected error occurred: {e}")

if __name__ == '__main__':
    main()
