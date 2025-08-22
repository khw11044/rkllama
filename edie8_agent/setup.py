from setuptools import find_packages, setup

package_name = 'edie8_agent'

setup(
    name=package_name,
    version='0.0.0',
    packages=[
        package_name,
        f"{package_name}.components",
        f"{package_name}.components.models",
        f"{package_name}.components.prompts",
        f"{package_name}.components.utils",
        f"{package_name}.components.toolbox",
    ],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='khw',
    maintainer_email='khw11044@naver.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'input = edie8_agent.input:main',
            'command = edie8_agent.command:main',
        ],
    },

)
