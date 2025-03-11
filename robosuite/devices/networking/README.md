# Networking

Networking module for communicating with controllers that sends custom messages.

## Protocol

Temporarily uses serialized JSON-encoded message.

    Offset  Length  Description
    ------  ------  -----------
    0       4       Length of entire framed message, as a little-endian 32-
                    bit unsigned integer, including these 4 bytes: N.
    4       N - 4   Payload as encoded by LaserTagJSONEncoder. The length is
                    the length given at offset 0 less 4 bytes (i.e., the JSON
                    payload itself).

## Acknowledgments

This module is based on the following sources. Please refer to the works below:

- [trzy's robot-arm](https://github.com/trzy/robot-arm)
