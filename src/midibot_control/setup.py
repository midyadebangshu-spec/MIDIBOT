from setuptools import setup

setup(
    name="midibot_control",
    version="0.1.0",
    packages=["midibot_control"],
    data_files=[("share/ament_index/resource_index/packages", ["resource/midibot_control"]),
                ("share/midibot_control", ["package.xml"])],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="MIDIBOT team",
    maintainer_email="midyadebangshu@gmail.com",
    description="MIDIBOT control nodes",
    license="MIT",
    entry_points={"console_scripts": ["hand_coupler = midibot_control.hand_coupler:main"]},
)
