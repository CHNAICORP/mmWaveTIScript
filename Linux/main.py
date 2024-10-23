import serial
import time
import subprocess
import os
import platform
from datetime import datetime, timedelta
import glob
import logging
import sys
import psutil
import re

def setup_logging():
    log_directory = './logs'
    log_name = 'mmwave.log'
    log_filename = os.path.join(log_directory, log_name)

    try:
        if not os.path.exists(log_directory):
            os.makedirs(log_directory)
            print(f"Created log directory at {log_directory}")
        else:
            print(f"Log directory already exists at {log_directory}")
        
        logging.basicConfig(filename=log_filename,
                            level=logging.INFO,
                            format='%(asctime)s - %(levelname)s - %(message)s',
                            datefmt='%Y-%m-%d %H:%M:%S')
        print(f"Logging setup complete. Log file at {log_filename}")
        logging.info("Logging setup successful.")
    except Exception as e:
        print(f"Error setting up logging: {e}")
        raise

def run_command(command):
    time.sleep(2)
    logging.info(f'Command executed: {command}')
    subprocess.run(command, shell=True)

def start_recording(command, quiet_mode=False):
    if quiet_mode:
        command += " -q"
    
    logging.info(f'Start recording with command: {command}')
    subprocess.Popen(command, shell=True)
    
    time.sleep(2)  # Give it some time to start

def kill_process_by_name(name):
    time.sleep(3)
    for proc in psutil.process_iter(['pid', 'name', 'cmdline']):
        try:
            if name in ' '.join(proc.info['cmdline']):
                logging.info(f'Killing process: {proc.info}')
                for child in proc.children(recursive=True):
                    child.kill()
                proc.kill()
                print(f"Killed process: {proc.info}")
                logging.info(f"Killed process: {proc.info}")
                break  # Found the process, no need to continue
        except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
            pass

def config_dca1000(quiet_mode=False):
    config_file = 'json/cf.json'
    command_prefix = './'
    executable = 'DCA1000EVM_CLI_Control'

    fpga_command = f"{command_prefix}{executable} fpga {config_file}"
    record_command = f"{command_prefix}{executable} record {config_file}"
    start_record_command = f"{command_prefix}{executable} start_record {config_file}"
    query_status_command = f"{command_prefix}{executable} query_status {config_file}"

    run_command(fpga_command)
    run_command(record_command)
    start_recording(start_record_command, quiet_mode=quiet_mode)
    run_command(query_status_command)

def config_iwr6843(serial_port):
    file_cfg = 'cfg/profile.cfg'
    print(f'Sending {file_cfg} to IWR6843AOP on {serial_port}')
    logging.info(f'Sending {file_cfg} to IWR6843AOP on {serial_port}')
    try:
        with serial.Serial(serial_port, 115200, timeout=1) as CLIport:
            with open(file_cfg, 'r') as file:
                config = [line.rstrip('\r\n') for line in file]
                for i in config:
                    if i == '' or i[0] == '%':
                        continue
                    current_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
                    CLIport.write((i + '\n').encode())
                    logging.info(f'[{current_time}] >>> {i}')
                    print(f'[{current_time}] >>> {i}')
                    if i == 'sensorStart':
                        break
                    time.sleep(0.1)
    except serial.SerialException as e:
        print(f"Serial port error: {e}")
        logging.info(f"Serial port error: {e}")

def stop_and_reset():
    config_file = 'json/cf.json'
    executable = 'DCA1000EVM_CLI_Control'

    current_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
    stop_record_command = f"./{executable} stop_record {config_file}"
    reset_fpga_command = f"./{executable} reset_fpga {config_file}"
    reset_ar_device_command = f"./{executable} reset_ar_device {config_file}"

    print(f'[{current_time}] Executing stop_record command...')
    logging.info(f'[{current_time}] Executing stop_record command...')
    run_command(stop_record_command)

    print(f'[{current_time}] Executing reset_fpga command...')
    logging.info(f'[{current_time}] Executing reset_fpga command...')
    run_command(reset_fpga_command)

    print(f'[{current_time}] Executing reset_ar_device command...')
    logging.info(f'[{current_time}] Executing reset_ar_device command...')
    run_command(reset_ar_device_command)

def extract_times_from_log(file_path):
    start_time = ""
    end_time = ""

    with open(file_path, 'r') as file:
        for line in file:
            if "Capture start time" in line:
                start_time = line.split(" - ")[1].strip()
            elif "Capture end time" in line:
                end_time = line.split(" - ")[1].strip()

    return start_time, end_time

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
    start_time = time.perf_counter()
    duration_seconds = duration / 1000.0  # Convert duration from milliseconds to seconds
    while True:
        elapsed_time = time.perf_counter() - start_time
        progress = elapsed_time / duration_seconds
        bar_length = 50
        block = int(round(bar_length * progress))
        text = f"\rProgress: [{'#' * block + '-' * (bar_length - block)}] {progress * 100:.2f}%"
        sys.stdout.write(text)
        sys.stdout.flush()
        if elapsed_time >= duration_seconds:
            break
        time.sleep(0.01)  # 10 milliseconds interval
    print()

def format_timedelta(td):
    seconds = td.total_seconds()
    milliseconds = int((seconds - int(seconds)) * 1000)
    seconds = int(seconds)
    minutes, seconds = divmod(seconds, 60)
    hours, minutes = divmod(minutes, 60)
    days, hours = divmod(hours, 24)
    return f"{days:02}:{hours:02}:{minutes:02}:{seconds:02}.{milliseconds:03}"

def main():
    setup_logging()
    data_dir = './data'
    serial_port_CLI = '/dev/ttyACM0'
    start_time = None
    end_time = None

    try:
        with serial.Serial(serial_port_CLI, 115200, timeout=1) as CLIport:
            sending = False
            quiet_mode = False  # You can enable or disable quiet mode as needed
    
            while True:
                if sending:
                    print("Program started. Enter 'q' to quit, 's' to stop collecting data, 'r' to reset mmWare Ladar.")
                    logging.info("Program started. Enter 'q' to quit, 's' to stop collecting data, 'r' to reset mmWare Ladar.")
                else:
                    print("Program started. Enter 'q' to quit, 's' to start automatically collecting data, 'r' to reset mmWare Ladar.")
                    logging.info("Program started. Enter 'q' to quit, 's' to start automatically collecting data, 'r' to reset mmWare Ladar.")

                user_input = input("Enter your command: ")

                if user_input.lower() == 'q':
                    if sending:
                        # kill_process_by_name('DCA1000EVM_CLI_Record')
                        stop_and_reset()                
                    break

                elif user_input.lower() == 's':
                    if sending:
                        log_file_path = find_log_file(data_dir)

                        if log_file_path:
                            start_time, end_time = extract_times_from_log(log_file_path)

                            if start_time and end_time:
                                rename_bin_files(data_dir, start_time, end_time)
                                print("Data collection completed!")
                                logging.info("Data collection completed!")
                            else:
                                print("Start time or end time not found in log file.")
                                logging.info("Start time or end time not found in log file.")
                        else:
                            print("Log file adc_data_Raw_LogFile.csv not found in the directory.")
                            logging.info("Log file adc_data_Raw_LogFile.csv not found in the directory.")
                        sending = False
                        # kill_process_by_name('DCA1000EVM_CLI_Record')
                        stop_and_reset()
                    else:
                        config_dca1000(quiet_mode=quiet_mode)
                        config_iwr6843(serial_port_CLI)
                        sending = True
                
                elif user_input.lower() == 'r':
                    stop_and_reset()
                    print("Reset mmWare Ladar finish!")
                    logging.info("Reset mmWare Ladar finish!")

    except serial.SerialException as e:
        print(f"Serial port error: {e}")
        logging.info(f"Serial port error: {e}")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
        logging.info(f"An unexpected error occurred: {e}")

if __name__ == '__main__':
    main()
