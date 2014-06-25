import numpy as np
#from matplotlib.path import Path

import matplotlib.pyplot as plt
import matplotlib.cbook as cbook

from  mpl_toolkits.axisartist.grid_helper_curvelinear import GridHelperCurveLinear
from mpl_toolkits.axisartist import Subplot

from mpl_toolkits.axisartist import SubplotHost, \
     ParasiteAxesAuxTrans

import  mpl_toolkits.axisartist.angle_helper as angle_helper
from matplotlib.projections import PolarAxes
from matplotlib.transforms import Affine2D

def curvelinear_test2(fig):
    """
    polar projection, but in a rectangular box.
    """

    # PolarAxes.PolarTransform takes radian. However, we want our coordinate
    # system in degree
    tr = Affine2D().scale(np.pi/180., 1.) + PolarAxes.PolarTransform()

    # polar projection, which involves cycle, and also has limits in
    # its coordinates, needs a special method to find the extremes
    # (min, max of the coordinate within the view).

    # 20, 20 : number of sampling points along x, y direction
    extreme_finder = angle_helper.ExtremeFinderCycle(50, 50,
                                                     lon_cycle = 360,
                                                     lat_cycle = None,
                                                     lon_minmax = None,
                                                     lat_minmax = (0, np.inf),
                                                     )

    grid_locator1 = angle_helper.LocatorDMS(12)
    # Find a grid values appropriate for the coordinate (degree,
    # minute, second).

    tick_formatter1 = angle_helper.FormatterDMS()
    # And also uses an appropriate formatter.  Note that,the
    # acceptable Locator and Formatter class is a bit different than
    # that of mpl's, and you cannot directly use mpl's Locator and
    # Formatter here (but may be possible in the future).

    grid_helper = GridHelperCurveLinear(tr,
                                        extreme_finder=extreme_finder,
                                        grid_locator1=grid_locator1,
                                        tick_formatter1=tick_formatter1
                                        )

    ax1 = SubplotHost(fig, 1, 1, 1, grid_helper=grid_helper)

    # make ticklabels of right and top axis visible.
    ax1.axis["right"].major_ticklabels.set_visible(True)
    ax1.axis["top"].major_ticklabels.set_visible(True)
    ax1.axis["left"].major_ticklabels.set_visible(True)

    # let right axis shows ticklabels for 1st coordinate (angle)
    ax1.axis["right"].get_helper().nth_coord_ticks=0
    # let bottom axis shows ticklabels for 2nd coordinate (radius)
    ax1.axis["bottom"].get_helper().nth_coord_ticks=0
    ax1.axis["left"].get_helper().nth_coord_ticks=0

    fig.add_subplot(ax1)


    ax1.quiver(0,0,50,50,angles='xy',scale_units='xy',scale=1)

    ax1.set_aspect(1.)
    ax1.set_xlim(-100, 100)
    ax1.set_ylim(-100, 100)

    ax1.grid(True)

if 1:
    fig = plt.figure(1, figsize=(7, 4))
    fig.clf()

    # curvelinear_test1(fig)
    curvelinear_test2(fig)

    plt.draw()
    plt.show()



