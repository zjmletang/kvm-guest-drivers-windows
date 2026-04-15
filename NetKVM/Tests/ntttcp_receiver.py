import socket
import subprocess
import time
import sys
import csv
import os
import platform
import xml.etree.ElementTree as ET
from datetime import datetime

# === Platform Detection ===
IS_WINDOWS = platform.system() == "Windows"
NTTTCP_CLIENT = "ntttcp.exe" if IS_WINDOWS else "ntttcp"

# === Configuration ===
SENDER_IP = sys.argv[1] if len(sys.argv) > 1 else "192.168.122.220"
TEST_MODE = sys.argv[2] if len(sys.argv) > 2 else "all"  # all, tcp-nonagle, tcp-nagle, udp
CONTROL_PORT = 9999
TEST_TIME = 10  # Set to 300 for formal testing (Microsoft recommended duration)

def get_my_ip():
    """Auto-detect local IP by checking which interface routes to SENDER_IP."""
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        s.connect((SENDER_IP, 1))  # UDP, no actual data sent
        return s.getsockname()[0]

MY_IP = get_my_ip()

# Linux only: network interface name for PPS stats (e.g. "virbr0", "eth0")
# Uses --show-nic-packets to get real network-layer packet counts.
# Set to empty string to disable.
INTERFACE_NAME = "" if IS_WINDOWS else "virbr0"

# Thread counts to test (iterate like iperf_wrapper's PARALLEL_THREADS)
THREAD_COUNTS = [1, 2, 4, 8, 16]

# Generate results filename with timestamp
TIMESTAMP = datetime.now().strftime("%Y%m%d_%H%M%S")
CSV_FILENAME = f"perf_results_{TIMESTAMP}.csv"

# Packet sizes to test
PACKET_SIZES = [32, 64, 128, 256, 512, 1024, 1460, 2048, 4096, 8192, 16384, 32768, 65536]
UDP_SIZES = [32, 64, 128, 256, 512, 1024, 1472]  # MTU-safe sizes

# Build scenarios based on test mode
def build_scenarios():
    scenarios = []
    if TEST_MODE in ("all", "tcp-nonagle"):
        for threads in THREAD_COUNTS:
            for size in PACKET_SIZES:
                scenarios.append(("TCP", size, threads, True))
    if TEST_MODE in ("all", "tcp-nagle"):
        for threads in THREAD_COUNTS:
            for size in PACKET_SIZES:
                scenarios.append(("TCP", size, threads, False))
    if TEST_MODE in ("all", "udp"):
        for threads in THREAD_COUNTS:
            for size in UDP_SIZES:
                scenarios.append(("UDP", size, threads, False))
    return scenarios

SCENARIOS = build_scenarios()

def wait_for_sender():
    """Wait for sender service to come online."""
    print(f"Waiting for sender service at {SENDER_IP}:{CONTROL_PORT}...")
    while True:
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                s.settimeout(3)
                s.connect((SENDER_IP, CONTROL_PORT))
                s.sendall(b"\n")
            print(f"Sender is online. Local IP: {MY_IP}\n")
            return
        except (ConnectionRefusedError, socket.timeout, OSError):
            time.sleep(2)

def signal_sender(proto, size, threads, no_nagle, duration):
    """Signals the remote sender with test parameters and duration."""
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.settimeout(5)
            s.connect((SENDER_IP, CONTROL_PORT))
            nagle_val = "1" if no_nagle else "0"
            s.sendall(f"{proto}:{size}:{threads}:{MY_IP}:{nagle_val}:{duration}".encode())
        return True
    except:
        return False

def get_sender_results():
    """Connect to sender and retrieve last test results (throughput:cpu:pps)."""
    for attempt in range(5):
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                s.settimeout(10)
                s.connect((SENDER_IP, CONTROL_PORT))
                s.sendall(b"RESULTS")
                data = s.recv(1024).decode().strip()
                parts = data.split(':')
                return float(parts[0]), float(parts[1]), int(parts[2])
        except Exception:
            time.sleep(2)
    return 0.0, 0.0, 0

def parse_stats_xml(xml_file):
    """Parse ntttcp XML output for throughput, CPU, and PPS."""
    mbps, cpu, pps = 0.0, 0.0, 0
    try:
        tree = ET.parse(xml_file)
        root = tree.getroot()
        realtime = float(root.findtext('realtime', '0'))
        for elem in root.findall('throughput'):
            if elem.get('metric') == 'mbps':
                mbps = round(float(elem.text), 2)
                break
        pkts = int(root.findtext('packets_received', '0'))
        if pkts > 0 and realtime > 0:
            pps = int(pkts / realtime)
        if IS_WINDOWS:
            cpu = round(float(root.findtext('cpu', '0')), 2)
        else:
            softirq = float(root.findtext('softirq', '0'))
            system = float(root.findtext('system', '0'))
            cpu = round(softirq + system, 2)
    except Exception as e:
        print(f"  XML parse error: {e}")
    return mbps, cpu, pps

# Main Execution
start_time = datetime.now()
wait_for_sender()
with open(CSV_FILENAME, mode='w', newline='') as csv_file:
    csv_writer = csv.writer(csv_file)
    csv_writer.writerow(["Protocol", "Size(B)", "Threads", "Nagle",
                         "Recv_Thr(Mbps)", "Recv_PPS", "Recv_CPU(%)",
                         "Send_Thr(Mbps)", "Send_PPS", "Send_CPU(%)"])

    print(f"### NetKVM Performance Report (Mode: {TEST_MODE}, Duration: {TEST_TIME}s)")
    print(f"### Results saved to: {CSV_FILENAME}")
    hdr = "| {:8} | {:7} | {:4} | {:8} | {:>9} | {:>7} | {:>6} | {:>9} | {:>7} | {:>6} |"
    sep = "| {:8} | {:7} | {:4} | {:8} | {:>9} | {:>7} | {:>6} | {:>9} | {:>7} | {:>6} |"
    print(hdr.format("Protocol", "Size(B)", "Thr", "Nagle",
                      "R_Thr", "R_PPS", "R_CPU%", "S_Thr", "S_PPS", "S_CPU%"))
    print(sep.format(":---", ":---", ":--", ":---", ":---", ":---", ":---", ":---", ":---", ":---"))

    for proto, size, threads, no_nagle in SCENARIOS:
        is_udp = "-u" if proto == "UDP" else ""
        nagle_str = "Disabled" if (proto == "TCP" and no_nagle) else ("Enabled" if proto == "TCP" else "N/A")
        recv_l = max(size, 65536) if proto == "UDP" else size
        xml_file = f"ntttcp_recv_{os.getpid()}.xml"
        if os.path.exists(xml_file):
            os.remove(xml_file)

        if IS_WINDOWS:
            rcv_cmd = f"{NTTTCP_CLIENT} -r {is_udp} -m {threads},*,{MY_IP} -l {recv_l} -t {TEST_TIME} -ns -xml {xml_file}"
        else:
            k_flag = f"--show-nic-packets {INTERFACE_NAME}" if INTERFACE_NAME else ""
            rcv_cmd = f"{NTTTCP_CLIENT} -r {is_udp} -P {threads} -N {k_flag} -x {xml_file} -t {TEST_TIME}"

        proc = subprocess.Popen(rcv_cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, universal_newlines=True)
        time.sleep(2)

        if signal_sender(proto, size, threads, no_nagle, TEST_TIME):
            try:
                stdout, stderr = proc.communicate(timeout=TEST_TIME + 30)
            except subprocess.TimeoutExpired:
                proc.kill()
                stdout, stderr = proc.communicate()
                print(f"  WARNING: ntttcp timed out")

            # Parse receiver stats
            if os.path.exists(xml_file):
                r_thr, r_cpu, r_pps = parse_stats_xml(xml_file)
            else:
                print(f"  WARNING: XML not generated. stderr: {stderr.strip()}")
                r_thr, r_cpu, r_pps = 0.0, 0.0, 0

            # Get sender stats
            s_thr, s_cpu, s_pps = get_sender_results()

            row = "| {:8} | {:7} | {:4} | {:8} | {:>9} | {:>7} | {:>6} | {:>9} | {:>7} | {:>6} |"
            print(row.format(proto, size, threads, nagle_str,
                             r_thr, r_pps, f"{r_cpu}%", s_thr, s_pps, f"{s_cpu}%"))
            csv_writer.writerow([proto, size, threads, nagle_str,
                                 r_thr, r_pps, r_cpu, s_thr, s_pps, s_cpu])
        else:
            print(f"| ERROR: Sender signal failed for {proto} {size}B |")
            proc.kill()

print(f"\nTest completed at {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}, total time: {datetime.now() - start_time}")
