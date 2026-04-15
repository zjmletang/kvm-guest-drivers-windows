import socket
import subprocess
import platform
import os
import xml.etree.ElementTree as ET

IS_WINDOWS = platform.system() == "Windows"
NTTTCP_CLIENT = "ntttcp.exe" if IS_WINDOWS else "ntttcp"

last_results = "0.0:0.0:0"

def parse_sender_xml(xml_file):
    """Parse sender's XML output for throughput, CPU, and PPS."""
    mbps, cpu, pps = 0.0, 0.0, 0
    try:
        tree = ET.parse(xml_file)
        root = tree.getroot()
        realtime = float(root.findtext('realtime', '0'))
        for elem in root.findall('throughput'):
            if elem.get('metric') == 'mbps':
                mbps = round(float(elem.text), 2)
                break
        pkts = int(root.findtext('packets_sent', '0'))
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

def start_sender_service():
    global last_results
    PORT = 9999
    xml_file = f"ntttcp_send_{os.getpid()}.xml"
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind(('0.0.0.0', PORT))
        s.listen(1)
        print(f"[*] NetKVM Performance Sender Service started (Port {PORT})...")
        while True:
            conn, addr = s.accept()
            with conn:
                data = conn.recv(1024).decode().strip()
                if not data:
                    continue

                # Return last test results
                if data == "RESULTS":
                    conn.sendall(last_results.encode())
                    continue

                parts = data.split(':')
                if len(parts) < 6:
                    continue

                proto, size, threads, target_ip, no_nagle, duration = parts
                is_udp = "-u" if proto == "UDP" else ""

                if IS_WINDOWS:
                    nagle_flag = "-ndl" if (proto == "TCP" and no_nagle == "1") else ""
                    cmd = f"{NTTTCP_CLIENT} -s -m {threads},*,{target_ip} {is_udp} {nagle_flag} -l {size} -t {duration} -ns -xml {xml_file}"
                else:
                    cmd = f"{NTTTCP_CLIENT} -s {is_udp} -P {threads} -N -t {duration} -x {xml_file} {target_ip}"

                print(f"[!] Executing: {cmd}")
                if os.path.exists(xml_file):
                    os.remove(xml_file)
                subprocess.run(cmd, shell=True, capture_output=True)

                mbps, cpu, pps = parse_sender_xml(xml_file)
                last_results = f"{mbps}:{cpu}:{pps}"
                print(f"[=] Send stats: Thr={mbps}Mbps CPU={cpu}% PPS={pps}")

if __name__ == "__main__":
    start_sender_service()
