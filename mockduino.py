'''
Mock serial port to simulate Arduino for testing

Copyright (c) 2023 Simon D. Levy

MIT License
'''


class MockArduino:

    def write(self, command):

        pass

    def readline(self):

        return ''

    def close(self):

        pass
