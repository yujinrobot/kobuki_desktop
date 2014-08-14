#!/usr/bin/env python
#       
# License: BSD
#   https://raw.github.com/yujinrobot/kobuki_desktop/master/kobuki_qtestsuite/LICENSE 
#
##############################################################################
# Imports
##############################################################################

import operator
import numpy

#from rqt_plot.qwt_data_plot import QwtDataPlot
from rqt_plot.data_plot import DataPlot

##############################################################################
# Classes
##############################################################################

class FullSizeDataPlot(DataPlot):
    def __init__(self, parent=None):
        super(FullSizeDataPlot, self).__init__(parent)
        self.max_range = 180
        self.min_range = 0
        self.dynamic_range = False
        self._ymin = 0
        self._ymax = 0
        
    def reset(self):
        self._ymin = 0
        self._ymax = 0
        
    ######################################
    # Overrides
    ######################################
    def _update_legend(self):
        handles, labels = self._canvas.axes.get_legend_handles_labels()
        if handles:
            hl = sorted(zip(handles, labels), key=operator.itemgetter(1))
            handles, labels = zip(*hl)
        self._canvas.axes.legend(handles, labels, loc='lower right')
        
    def redraw(self):
        ''' 
          We fix the y axis and continually resize the x axis to encapsulate
          the entire domain, range of the battery profile.
          
          @Todo : the domain is simply the data value, we could use
        '''
        self._canvas.axes.grid(True, color='gray')
        # Set axis bounds
        xmax = 0
        for curve in self._curves.values():
            data_x, data_y, plot, min_max_y = curve
            if len(data_x) == 0:
                continue
            
            xmax = max(xmax, data_x[-1])
            self._ymin = min(self._ymin, min_max_y[0])
            self._ymax = max(self._ymax, min_max_y[1] + 1)
            self._canvas.axes.set_xbound(lower=0, upper=xmax)
            
        if self.dynamic_range:
            self._canvas.axes.set_ybound(lower=self._ymin, upper=self._ymax)
        else:
            self._canvas.axes.set_ybound(self.min_range, upper=self.max_range)

        # Set plot data on current axes
        for curve in self._curves.values():
            data_x, data_y, plot, min_max_y = curve
            plot.set_data(numpy.array(data_x), numpy.array(data_y))

        self._canvas.draw()
