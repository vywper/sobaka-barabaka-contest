"""Запускает публичный симулятор, Participant и проверку в Docker."""

import argparse
from pathlib import Path
import subprocess
import sys
import uuid


SIMULATOR = (
    "registry.cookcloud.ru/go1-hackathon-remote-stage/go1-sim@sha256:"
    "c79ddbece6cc7d00f114a9a9a09be0bae061ebb35ca722bcbe51bf75fce1b2e3"
)
DDS_CONFIG = """<CycloneDDS xmlns="https://cdds.io/config"><Domain id="any">
<General><Interfaces><NetworkInterface name="lo" multicast="true"/></Interfaces>
<AllowMulticast>spdp</AllowMulticast><EnableMulticastLoopback>true</EnableMulticastLoopback>
</General><Discovery><ParticipantIndex>none</ParticipantIndex></Discovery>
</Domain></CycloneDDS>"""
TESTS = Path(__file__).resolve().parent


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--context", default="desktop-linux" if sys.platform == "win32" else "default")
    parser.add_argument("--image", default="go1-shotigolova:t2-local")
    args = parser.parse_args()
    docker = ["docker", "--context", args.context]
    run_id = uuid.uuid4().hex[:10]
    simulation = f"go1-t2-sim-{run_id}"
    participant = f"go1-t2-participant-{run_id}"
    probe = f"go1-t2-probe-{run_id}"
    common = ["--pull", "never", "--platform", "linux/amd64", "-e", "ROS_DOMAIN_ID=51",
              "-e", "RMW_IMPLEMENTATION=rmw_cyclonedds_cpp", "-e", "ROS_LOCALHOST_ONLY=0",
              "-e", f"CYCLONEDDS_URI={DDS_CONFIG}"]

    subprocess.run(docker + ["image", "inspect", args.image], stdout=subprocess.DEVNULL, check=True)
    available = subprocess.run(docker + ["image", "inspect", SIMULATOR], capture_output=True)
    if available.returncode:
        subprocess.run(docker + ["pull", "--platform", "linux/amd64", SIMULATOR], check=True)

    # loopback даёт обмен DDS между контейнерами, даже с --network none у симулятора.
    try:
        subprocess.run(docker + ["run", "-d", "--name", simulation, "--network", "none",
                                 "-e", "GO1_DDS_LOCAL=1", *common, SIMULATOR,
                                 "ros2", "launch", "go1_simulation", "scenario.launch.py",
                                 "scenario:=lowlevel", "gui:=false"], check=True)
        subprocess.run(docker + ["run", "-d", "--name", participant, "--network",
                                 f"container:{simulation}", *common, args.image, "T2"], check=True)
        subprocess.run(docker + ["run", "--name", probe, "--network", f"container:{simulation}",
                                 *common, "--mount", f"type=bind,source={TESTS},target=/t2-tests,readonly",
                                 "--entrypoint", "/bin/bash", args.image, "-ec",
                                 "source /opt/ros/humble/setup.bash; "
                                 "source /opt/go1_interfaces/local_setup.bash; "
                                 "exec python3 /t2-tests/t2_sim_probe.py"], check=True, timeout=120)
    except (subprocess.CalledProcessError, subprocess.TimeoutExpired):
        for name in (participant, simulation):
            print(f"\nЛоги {name}", file=sys.stderr, flush=True)
            subprocess.run(docker + ["logs", "--tail", "50", name], timeout=10)
        return 1
    finally:
        subprocess.run(docker + ["rm", "-f", probe, participant, simulation], capture_output=True, timeout=30)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
