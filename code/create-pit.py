import numpy as np
import sys
from scipy.stats import truncnorm
import matplotlib.pyplot as plt

def get_truncated_normal(mean=0, sd=1, low=0, upp=10):
    return truncnorm(
        (low - mean) / sd, (upp - mean) / sd, loc=mean, scale=sd)

# Used from: https://stackoverflow.com/questions/16750618/whats-an-efficient-way-to-find-if-a-point-lies-in-the-convex-hull-of-a-point-cl
def in_hull(p, hull):
        """
        Test if points in `p` are in `hull`

        `p` should be a `NxK` coordinates of `N` points in `K` dimensions
        `hull` is either a scipy.spatial.Delaunay object or the `MxK` array of the 
        coordinates of `M` points in `K`dimensions for which Delaunay triangulation
        will be computed
        """
        if not isinstance(hull,Delaunay):
                hull = Delaunay(hull)
        return hull.find_simplex(p)>=0

def createGridCoordinates(start_x, start_y, end_x, end_y):
        x = np.arange(start_x,end_x+1)
        y = np.arange(start_y,end_y+1)

        xtiled = np.tile(x, (y.shape[0],1))
        ytiled = np.tile(y, (x.shape[0],1))

        coords = np.stack((xtiled,ytiled.T),axis=2)
        return coords.reshape(-1,2)

def findPtsInHull(hull_points, start_x, start_y, end_x, end_y):
        coords = createGridCoordinates(start_x, start_y, end_x, end_y)
        inside = in_hull(coords, hull_points)
        return coords[np.where(inside==True)]
        
if __name__ == "__main__":
    corners = 80#sys.argv[1]datada
    approx_radius = 25#sys.argv[2]

    total_radius_delta = 0.2*approx_radius
    adjecent_delta = 0.1*approx_radius

    corner_angles = np.linspace(0,2*np.pi,corners+1)
    r = np.zeros_like(corner_angles)
    x = np.zeros_like(corner_angles)
    y = np.zeros_like(corner_angles)
    x_new = np.zeros_like(corner_angles)

    X = get_truncated_normal(0,0.3,-1,1)
    s = X.rvs(corners)
    s = np.hstack((np.array([0]),s))

    r[0] = approx_radius
    x[0] = r[0]*np.cos(corner_angles[0])
    y[0] = r[0]*np.sin(corner_angles[0])
    for i in range(1,corners+1):
        r[i] = r[i-1] + adjecent_delta*s[i]
        x[i] = r[i]*np.cos(corner_angles[i])
        y[i] = r[i]*np.sin(corner_angles[i])
    diff = x[corners]-x[0]
    print(x[corners]-x[0])
    x_new = x.copy()
    halfway = int(np.ceil((corners+1)/2))
    for j in range(halfway,corners+1):
        x_new[j] = x_new[j] - diff*(j-halfway)/np.floor(corners/2)
    r[i] = approx_radius
    x[i] = r[i]*np.cos(corner_angles[0])
    x_new[i] = r[i]*np.cos(corner_angles[0])
    y[i] = r[i]*np.sin(corner_angles[0])

    # plt.plot(x,y)
    # plt.plot(x_new,y)
    # plt.axis('equal')
    # plt.savefig("data/pit-"+str(corners)+"-"+str(adjecent_delta)+"-"+str(np.random.randint(0,100))+".png")
    # plt.show()
