#!/usr/bin/env python3

#quick cameraserver

from cscore import CameraServer

def main():
    cs = CameraServer.getInstance()
    cs.enableLogging()

    cs.startAutomaticCapture()
    cs.waitForever()


if __name__ == "__main__":

    import logging

    logging.basicConfig(level=logging.DEBUG)

    import networktables
    networktables.NetworkTables.initialize(server='10.42.53.2')

    main()

