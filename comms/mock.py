'''
Mock Arduino communications for testing

Copyright (c) 2023 Simon D. Levy

MIT License
'''


class Comms:

    def write(self, command):

        pass

    def readline(self):

        return ''

    def close(self):

        pass
