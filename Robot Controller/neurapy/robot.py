import json
import socket
import datetime
from types import MethodType


import logging
import sys
import os
from logging.handlers import TimedRotatingFileHandler
from threading import Thread
import time
from prettytable import PrettyTable

OS = None
if sys.platform == "linux":
    import signal
    OS = "linux"
else :
    import win32api
    OS = "windows"

VERSION = "v5.0.8"
    
LOGLEVEL = os.getenv('NEURAPY_LOG_LEVEL','WARNING')

MONITOR_CYCLE_TIME = 2

SOCKET_ADDRESS = os.getenv("SOCKET_ADDRESS", "192.168.2.13")
SOCKET_PORT = 65432

class CustomFormatter(logging.Formatter):

    grey = "\x1b[38;20m"
    yellow = "\x1b[33;20m"
    red = "\x1b[31;20m"
    bold_red = "\x1b[31;1m"
    reset = "\x1b[0m"
    format = (
        "[%(asctime)s][%(name)s][%(levelname)s] : %(message)s :(%(filename)s:%(lineno)d)"
    )

    FORMATS = {
        logging.DEBUG: grey + format + reset,
        logging.INFO: grey + format + reset,
        logging.WARNING: yellow + format + reset,
        logging.ERROR: red + format + reset,
        logging.CRITICAL: bold_red + format + reset,
    }

    def format(self, record):
        log_fmt = self.FORMATS.get(record.levelno)
        formatter = logging.Formatter(log_fmt,datefmt="%Y-%m-%d %H:%M:%S")
        return formatter.format(record)


def get_console_handler():
    console_handler = logging.StreamHandler(sys.stdout)
    console_handler.setLevel(eval('logging.'+LOGLEVEL))
    console_handler.setFormatter(CustomFormatter())
    return console_handler


def get_logger(logger_name):
    logger = logging.getLogger(logger_name)
    if not logger.hasHandlers():
        logger.addHandler(get_console_handler())
    logger.setLevel(logging.DEBUG)
    logger.propagate = False
    return logger

neurapy_logger = get_logger("neurapy_logger")

def receive_json(socket, buffer_size=8192):
    data = ""
    attempt = 0
    while attempt <= 5:
        received_data = socket.recv(buffer_size).decode('utf-8')
        if not received_data:
            break
        data += received_data
        try:
            # Try to decode the JSON data
            result = json.loads(data)
            return result
        except json.JSONDecodeError:
            if attempt == 5:
                raise Exception('Could not receive data even after 5 attempts')
            attempt +=1
            # If JSON is not complete, continue receiving
            continue



def generate_function(function_name, address):
    def wrapped_function(self, *args, **kwargs):
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            sock.connect(address)
        except ConnectionError:
            raise ConnectionError("Failed to establish the communication to control box. Cross-check whether the robot is reachable or try reset control from Teach pendant")
        try:
            if OS == 'linux':
                signal.signal(signal.SIGINT,lambda signal, frame: self.stop())
                signal.signal(signal.SIGHUP,lambda signal, frame: self.stop())
            else:
                win32api.SetConsoleCtrlHandler(self.stop, True)
        except Exception as e:
            self.logger.debug("Not attaching signal handlers")
            
        self.logger.info(f"{function_name} called with args {args}, {kwargs}")
        data = {"function": function_name, "args": args, "kwargs": kwargs}
        sock.sendall(json.dumps(data).encode("utf-8"))
        response = receive_json(sock,8192)
        sock.close()
        if response["error"]:
            self.logger.error(f"{function_name} call with args {args}, {kwargs}, failed with exception {response['error']}")
            raise Exception(response["error"])
        return response["result"]

    wrapped_function.__name__ = function_name + "_method"
    return wrapped_function


class Robot:
    def __init__(self):
        self.__server_address = (SOCKET_ADDRESS, SOCKET_PORT)
        self.__functions = ["get_functions","initialize_attributes"]
        self.counter = 0
        self.multiplier = 1
        self.logger = neurapy_logger
        for function in self.__functions:
            setattr(
                self,
                function,
                MethodType(generate_function(function, self.__server_address), self),
            )

        for key, value in self.initialize_attributes().items():
            setattr(self, key, value)
        for method in self.get_functions():
            if method != "list_methods":
                setattr(
                    self,
                    method,
                    MethodType(generate_function(method, self.__server_address), self),
                )     
        self.logger.info(f"Robot initialized with following functions {self.__functions} and robot version {self.version}")
        if self.version != VERSION:
            self.logger.warning(f"Current client version is not compatiable with the version of the server running on the robot. Some of the functionlities specified in the documentation might not work in the intended way. Upgrade to the correct version .Client Version : {VERSION},Server Version : {self.version}")
        self.start_diagnostics_monitor()
        
    def help(self, name):
        print(self.get_doc(name))
        
    def list_methods(self):
        """To list the available functions in the API"""
        methods = self.get_functions()
        table = PrettyTable()
        table.field_names = ['S.No',"Function Name"]
        for index,method in enumerate(methods):
            table.add_row([index+1,method])
        return table
        
    def notify_diagnostics(self):
        while True:
            diagnostics = self.get_diagnostics()
            if "critical" in diagnostics:
                if diagnostics["critical"]:
                    self.counter += 1
                    if self.counter > 1000*self.multiplier:
                        self.logger.error(f"{diagnostics}")
                        self.counter = 0
                        self.multiplier += 1
            time.sleep(MONITOR_CYCLE_TIME)
        
    def start_diagnostics_monitor(self):
        self.monitor = Thread(target=self.notify_diagnostics)
        self.monitor.daemon = True
        self.monitor.start()