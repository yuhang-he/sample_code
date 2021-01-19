from __future__ import print_function
import grpc

import src.protos.scanner_commands_pb2 as scanner_commands_pb2
import src.protos.scanner_commands_pb2_grpc as scanner_commands_pb2_grpc
import subprocess
import multiprocessing

from PySide2.QtCore import QThread, Signal


def subproc():
    """Start a subprocess to execute FaroScanServer.exe"""
    subprocess.check_call('FaroScanServer.exe', shell=True,
                          cwd='C:\\Users\\yuhhe\\source\\repos\\FaroScanServer\\FaroScanServer\\bin\\x64\\Release',
                          close_fds=True)


class ScannerUpdate(QThread):
    """A thread to monitor scan progress"""
    progress_update = Signal()
    def __init__(self, scanner):
        QThread.__init__(self)
        self.scanner = scanner

    def run(self):
        """A loop to monitor scan progress until it is complete"""
        self.scanner.scan_progress = 0
        while self.scanner.scanning and self.scanner.scan_progress < 100:
            response_future = self.scanner.client.ScanProgress.future(scanner_commands_pb2.ScannerRequest(request=1))
            progress = response_future.result()
            self.scanner.scan_progress = progress.ScanProgress
            self.scanner.scanning = not progress.ScanCompleted
            self.progress_update.emit()


class ScannerControl(object):
    """A class to control scanner through gRPC communication"""
    def __init__(self, window):
        self._window = window
        self._window.enable_scanner.clicked.connect(self._enable)
        self._window.start_scanner.clicked.connect(self._control)
        self._window.reset_scanner.clicked.connect(self._reset)
        self._window.scan_connect.clicked.connect(self._connect)
        self._window.scan_start.clicked.connect(self._start)
        self._window.scan_stop.clicked.connect(self._stop)
        self._window.scan_shutdown.clicked.connect(self._shutdown)

        self.subthread = ScannerUpdate(self)
        self.subthread.progress_update.connect(self.progress_update)

        self.channel = None
        self.client = None
        self.scanning = False
        self.scan_progress = 0

    @staticmethod
    def _enable(self):
        """Execute FaroScanServer.exe and create a gRPC client to communicate with the C# app"""
        sub = multiprocessing.Process(target=subproc)
        sub.start()

    def _control(self):
        """Create a gRPC channel at localhost and port 50051"""
        self.channel = grpc.insecure_channel('localhost:50051')
        self.client = scanner_commands_pb2_grpc.ScannerCommandStub(self.channel)

    def _reset(self):
        self.scanning = False

    def _connect(self):
        """Connect to scanner using the IP address entered in GUI"""
        def process_response(future):
            response = future.result()
            self._window.qtlog.append(response.ErrorResponse.Name(response.error_response)[14:])
            self._window.qtlog.append("Scanner Connect")

        response_future = self.client.ScanConnect.future(scanner_commands_pb2.IpAddress
                                                         (ScannerIP=self._window.scan_ip.text()))
        response_future.add_done_callback(process_response)

    def _start(self):
        """Parse parameter information entered in the GUI and send the params to server through gRPC"""
        def process_response(future):
            response = future.result()
            self._window.qtlog.append(response.ErrorResponse.Name(response.error_response)[14:])
            self.scanning = True
            self.subthread.start()
            self._window.qtlog.append("Scanner Start")

        param = scanner_commands_pb2.ScanParam(
            ScannerIP=self._window.scan_ip.text(),
            ScanMode=self._window.scan_mode.currentText(),
            VerticalAngleMin=int(self._window.scan_v_min.text()),
            VerticalAngleMax=int(self._window.scan_v_max.text()),
            HorizontalAngleMin=int(self._window.scan_h_min.text()),
            HorizontalAngleMax=int(self._window.scan_h_max.text()),
            Resolution=int(self._window.scan_resolution.currentText()),
            MeasurementRate=int(self._window.scan_measure_rate.currentText()),
            NoiseCompression=int(self._window.scan_noise_comp.currentText()),
            ScanFileNumber=int(self._window.scan_f_num.text()),
            ScanBaseName=self._window.scan_f_name.text(),
            StorageMode=self._window.scan_storage.currentText(),
            RemoteScanStoragePath=self._window.scan_path.text()
        )
        response_future = self.client.StartScan.future(param)
        response_future.add_done_callback(process_response)

    def _stop(self):
        """Send a stop request through gRPC"""
        def process_response(future):
            response = future.result()
            self._window.qtlog.append(response.ErrorResponse.Name(response.error_response)[14:])
            self.scanning = False
            self._window.qtlog.append("Scanner Stop")

        response_future = self.client.StopScan.future(scanner_commands_pb2.ScannerRequest(request=1))
        response_future.add_done_callback(process_response)

    def _shutdown(self):
        """Send a shutdown request through gRPC"""
        def process_response(future):
            response = future.result()
            self._window.qtlog.append(response.ErrorResponse.Name(response.error_response)[14:])
            self._window.qtlog.append("Scanner ShutDown")

        response_future = self.client.ScanShutDown.future(scanner_commands_pb2.ScannerRequest(request=1))
        response_future.add_done_callback(process_response)

    def progress_update(self):
        """Display the scan progress on the GUI screen"""
        self._window.scan_progress.setValue(self.scan_progress)
