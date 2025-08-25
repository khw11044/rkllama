from setuptools import find_packages, setup

package_name = 'edie8_agent_emb'

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
        f"{package_name}.components.rkllama_core",
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
            'input_emb = edie8_agent_emb.input_emb:main',
            'command_emb = edie8_agent_emb.command_emb:main',
        ],
    },

)
