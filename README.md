# JETPACK CAMERA API: LIBARGUS

## Design Goals
Open Standard

Cross Platform

Low-level control of camera subsystem

Frame-based capture control

Metadata output for frame statistics

Multi-stream, multi-camera, multi-process support

Efficient GPU processing and API interop via EGLStreams

Extendable and backwards compatible


## Coding Standards
Argus:: namespace

C++03

No exceptions

No RTTI

## Objects and Interfaces
Objects do not have methods. All methods are provided by Interfaces.

### Objects
Unique handle to API entity

All objects are InterfaceProviders

Two types of objects:
1. Destructable: created, owned, and destroyed by the client
2. Non-Destructable: children of other libargus objects; owned and destroyed by
parent object.

### Interfaces
Pure virtual class

Name prefixed with ‘I’ (ie. IEvent)

Identified by 128 bit UUID, InterfaceID

Acquired at runtime from an InterfaceProvider (ie. Object)

Valid throughout lifetime of object

Interfaces do not change once published*

New functionality added using new interfaces*

*Interfaces still subject to change before libargus 1.0 release




