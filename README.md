# Continuum

Continuum is a middleware that unifies diverse voice and LLM services behind a single WebSocket interface,
with a pluggable architecture that makes it easy to swap providers or add new capabilities.

## Getting Started

Typically, to integrate Continuum into your project, you would run the Continuum server locally using Docker and
interact with it using the provided [CLI](CLI.md) or [Python client](./lib).

Continuum relies on ROS 2 (Robot Operating System) as its pubsub framework. This approach takes
advantage of existing ROS tooling and simplifies integration with hardware projects. See the [workspace](./workspace)
folder for a peek under the hood or if you want to contribute to the project.

## Design Principles

- Every feature should have two or more implementations, whenever possible. We believe it's important to have
  interfaces that are not tightly coupled to a specific product or provider to reduce any potential vendor lock-in.

- Every feature should have a local implementation and be the default option, whenever possible. We believe it's
  important to be able to build applications where we are in full control of the data.

- We understand that cloud options can provide the right trade-offs for some users (e.g., accessibility, hardware
  constraints, quality, etc.) and such options are also included and supported.

- Generally, we favor integrating solutions that are multilingual, multimodal, low-latency, and open source.

## Why ROS

ROS might seem like an odd choice for building an application server. However, it's worth considering that:

- To build any non-trivial application, a framework is always needed. ROS aligns well with the design principles
  outlined above and provides a solution that is flexible, low-latency, well-tested, documented, and open source.

- Most voice and AI providers default to Python implementations. Python is a first-class citizen in ROS.

- Beyond powering the desktop applications mentioned below, we envision this project supporting custom hardware 
  interfaces. ROS provides an excellent bridge for hardware projects given its robotics foundation.

- Both the ROS native bridge and Foxglove provide a low-latency WebSocket interface that allows any developer to
  interact with the underlying pubsub functionality in any programming language, regardless of ROS knowledge.

## Built with Continuum

Currently, Continuum powers two other projects by the same author:
[Speed of Sound](https://github.com/zugaldia/speedofsound) and
[Speed of Light](https://github.com/zugaldia/speedoflight).

## License

This project is licensed under the [MIT License](LICENSE).
