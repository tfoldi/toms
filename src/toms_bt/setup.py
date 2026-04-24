from glob import glob

from setuptools import find_packages, setup

package_name = "toms_bt"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="TODO",
    maintainer_email="todo@example.com",
    description="TOMS behavior tree runtime and node definitions",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "run_task = toms_bt.runner:main",
            "run_task_instrumented = toms_bt.instrumented_runner:main",
            "run_preflight = toms_bt.preflight_node:main",
        ],
    },
)
