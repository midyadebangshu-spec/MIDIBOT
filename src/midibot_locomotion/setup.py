from setuptools import setup

setup(
    name="midibot_locomotion",
    version="0.1.0",
    packages=["midibot_locomotion"],
    data_files=[("share/ament_index/resource_index/packages", ["resource/midibot_locomotion"]),
                ("share/midibot_locomotion", ["package.xml"])],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="MIDIBOT team",
    maintainer_email="midyadebangshu@gmail.com",
    description="MIDIBOT walking library",
    license="Proprietary",
)
