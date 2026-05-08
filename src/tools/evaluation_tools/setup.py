from setuptools import setup

package_name = "evaluation_tools"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/scripts", ["scripts/bag_replay_report.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Nithish",
    maintainer_email="nithish@example.com",
    description="Evaluation and replay helpers for JetAuto Orin AMR.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "bag_replay_report = evaluation_tools.bag_replay_report:main",
        ],
    },
)

