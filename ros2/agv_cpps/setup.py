from setuptools import setup, find_packages
import os
from glob import glob

package_name = "agv_cpps"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="user",
    maintainer_email="user@example.com",
    description="CPPS AGV replenishment simulation nodes",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "order_manager = agv_cpps.order_manager:main",
            "fleet_manager  = agv_cpps.fleet_manager:main",
            "agv_controller = agv_cpps.agv_controller:main",
            "event_logger   = agv_cpps.event_logger:main",
        ],
    },
)
