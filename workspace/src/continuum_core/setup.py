from setuptools import find_packages, setup

package_name = "continuum_core"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=[
        "setuptools",
        "continuum",
    ],
    zip_safe=True,
    maintainer="Antonio Zugaldia",
    maintainer_email="antonio@zugaldia.com",
    description="Continuum core package",
    license="MIT",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            # Health nodes
            "heartbeat_node = continuum_core.health.heartbeat_node:main",
            "echo_node = continuum_core.health.echo_node:main",
            # Joystick nodes
            "joystick_node = continuum_core.input.joystick_node:main",
            # ASR nodes
            "fake_asr_node = continuum_core.asr.fake_asr_node:main",
            "fasterwhisper_asr_node = continuum_core.asr.fasterwhisper_asr_node:main",
            "openai_asr_node = continuum_core.asr.openai_asr_node:main",
            # LLM nodes
            "fake_llm_node = continuum_core.llm.fake_llm_node:main",
            "ollama_llm_node = continuum_core.llm.ollama_llm_node:main",
            "openai_llm_node = continuum_core.llm.openai_llm_node:main",
            "google_llm_node = continuum_core.llm.google_llm_node:main",
            "mapgpt_llm_node = continuum_core.llm.mapgpt_llm_node:main",
            # TTS nodes
            "kokoro_tts_node = continuum_core.tts.kokoro_tts_node:main",
            "elevenlabs_tts_node = continuum_core.tts.elevenlabs_tts_node:main",
            # App nodes
            "dictation_app_node = continuum_core.apps.dictation_app_node:main",
            # Reachy
            "reachy_node = continuum_core.reachy.reachy_node:main",
        ],
    },
)
