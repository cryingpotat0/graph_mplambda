class Logger():
    PRINT = 'print'
    FILE = 'file'
    def __init__(self, typ='print', **kwargs):
        """
        type: ['print', 'file']
        """
        if typ == self.PRINT:
            self._typ = self.PRINT
        elif typ == self.FILE:
            self._typ = self.FILE
            self._filename = kwargs['filename']
            self._num_lines_before_flush = kwargs.get('num_lines_before_flush') or 10
            self._lines = []
        else:
            raise ValueError("Invalid Logger type {}".format(self._typ))

    def log(self, item,):
        if self._typ == self.PRINT:
            print(item)
        elif self._typ == self.FILE:
            if len(self._lines) > self._num_lines_before_flush:
                # TODO: log to file
                pass


    def shutdown(self, ):
        pass


