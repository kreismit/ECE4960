import numpy as np
from matplotlib import pyplot as plt
import cv2
import random
import math

def display_grid(grid):
    """Display a 2D binary numpy matrix as a grid image 

    Args:
        grid ([ndarray]): A 2D binary matrix
    """
    uint_img = np.array(grid*255).astype('uint8')
    grayImage = cv2.cvtColor(uint_img, cv2.COLOR_GRAY2BGR)
    plt.imshow(grayImage, interpolation='nearest')
    plt.show()

class PlannerQuery():
    """A class to generate a start and goal cell in 
       a 2D binary occupancy grid. The start and goal cell
       should be:
       - within the contour of the enclosed space, 
       - reasonably far apart
       - should be obstacle-free 
       - such that the required plan should not be trivial.
    """
    def __init__(self, grid):
        """Initialize 

        Args:
            grid ([ndarray]): A 2D binary matrix

        Raises:
            Exception: If 2D matrix
            Exception: If binary matrix
        """
        self.grid_sanity_check(grid)
        
        self.grid = grid
        self.size = self.grid.size
        self.shape = self.grid.shape
        
        self.MULTIPLIER = 1/4
        self.MIN_DIST = math.hypot(self.shape[0],self.shape[1]) * self.MULTIPLIER
        self.MAX_GOAL_RETRIES = 500
        self.MAX_FREE_CELL_RETRIES = 50
        
        self.get_mask()
    
    def grid_sanity_check(self, grid):
        """Sanity check the grid. 

        Args:
            grid ([ndarray]): A 2D binary matrix

        Raises:
            Exception: If 2D matrix
            Exception: If binary matrix
        """
        # Check if 2D matrix
        if(grid.ndim != 2):
            raise Exception("The grid is not a 2D numpy matrix")
        
        # Check if binary matrix
        if (np.array_equal(grid, grid.astype(bool)) == False):
            raise Exception("The 2D grid is not a binary matrix")
            
    def get_mask(self, plot=False):
        """Get free space mask for sampling free cells

        Args:
            plot (bool, optional): Set to True to plot the mask and print debug info. Defaults to False.
        """
        self.mask = np.zeros_like(self.grid)
        self.img_mask = cv2.cvtColor(np.zeros_like(self.grid), cv2.COLOR_GRAY2RGB)

        uint_img = np.array(self.grid*255).astype('uint8')
        grayImage = cv2.cvtColor(uint_img, cv2.COLOR_GRAY2BGR)
        contours, hierarchy = cv2.findContours(uint_img,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(self.img_mask, contours, 0, (255, 0, 0), cv2.FILLED)

        for i in range(0,self.grid.shape[0]):
            for j in range(0,self.grid.shape[1]):
                if(cv2.pointPolygonTest(contours[0],(j,i),True) > 0):
                    self.img_mask[i,j,:] = [0, 255, 0]
                    self.mask[i,j] = 1
        if(plot):
            plt.imshow(self.img_mask)
            plt.show()

    def get_random_index(self):
        """Get a random free cell index within the contour

        Returns:
            Exception: If couldn't find a valid free cell within MAX_FREE_CELL_RETRIES retries
        """
        mask_1d = self.mask.ravel()

        dist = np.arange(self.size)
        p_dist = mask_1d/np.sum(mask_1d)

        for i in range(0,self.MAX_FREE_CELL_RETRIES):
            idx = np.unravel_index(np.random.choice(dist, p=p_dist), self.shape)
            if(self.is_free_cell(*idx)):
                return idx

        raise Exception("Couldn't find a valid start/goal cell. Please call generate() again.")
        
    def get_start_cell(self):
        return self.get_random_index()

    def n_closest(sef, x, n, d=1):
        return x[n[0]-d:n[0]+d+1,n[1]-d:n[1]+d+1]

    def set_n_closest(self, x, n, d=1, c=255):
        x[n[0]-d:n[0]+d+1,n[1]-d:n[1]+d+1] = c
    
    def get_line_indices(self, start_cell, goal_cell):
        """Get indices of the line segment between the start and goal cells

        Args:
            start_cell [(int, int)]: Start cell in the grid
            start_cell [(int, int)]: Goal cell in the grid

        Returns:
            [(ndarray, ndarray)]: A tuple of indices of the line segment
        """
        number_of_points=max(abs(goal_cell[0]-start_cell[0]), abs(goal_cell[1]-start_cell[1])) + 3
        # print("Number of points: ", number_of_points)
        
        xs=np.rint(np.linspace(start_cell[0],goal_cell[0],number_of_points)).astype(int)
        ys=np.rint(np.linspace(start_cell[1],goal_cell[1],number_of_points)).astype(int)
        
        return (xs,ys)
    
    def is_free_cell(self,i,j):
        return (self.grid[i,j] == 0)
    
    def valid_goal(self, start_cell, goal_cell):
        """Check the validity of the goal cell.
        It checks if the distance between the cells is greater than MIN_DIST.
        If they are almost collinear, it checks if there is an obstacle between 
        them.

        Args:
            start_cell [(int, int)]: Start cell in the grid
            goal_cell [(int, int)]: Goal cell in the grid

        Returns:
            [bool]: True if valid, False otherwise
        """
        dy = abs(goal_cell[1] - start_cell[1])
        dx = abs(goal_cell[0] - start_cell[0])
        if(math.hypot(dy,dx) > self.MIN_DIST):
            if(min(dx,dy) < 2):
                if(np.any(self.grid[self.get_line_indices(start_cell, goal_cell)]) == 1):
                    # print(" | Obstacle In between")
                    return True
                else:
                    return False
            else:
                return True
        
        else:
            return False

    def get_goal_cell(self, start_cell):
        """Randomly sample a goal cell within the contour and check for its validity 
        until MAX_GOAL_RETRIES retries fail

        Args:
            start_cell [(int, int)]: Start cell in the grid

        Returns:
            goal_cell [(int, int)]: Goal cell in the grid

        Raises:
            Exception: If no valid cell is found after MAX_GOAL_RETRIES attempts
        """
        for i in range(0,self.MAX_GOAL_RETRIES):
            goal_cell = self.get_random_index()
            if(self.valid_goal(start_cell, goal_cell)):
                return goal_cell

        raise Exception("Couldn't find a valid goal cell for the given start cell. Please call generate() again.")
     
    def generate(self, plot = False):
        """Generate a valid start and goal cell for the planner

        Args:
            plot (bool, optional): Se this to true to plot and print debug info. Defaults to False.

        Returns:
            start_cell [(int, int)]: Start cell in the grid
            goal_cell [(int, int)]: Goal cell in the grid
        """
        output_img = cv2.cvtColor(np.copy(self.grid)*255,cv2.COLOR_GRAY2RGB)

        start_cell = self.get_start_cell()
        goal_cell = self.get_goal_cell(start_cell)
        
        if plot == True:
            # set_n_closest(output_image, start_cell, 3, [0,0,255])
            
            print("Start cell: ", start_cell)
            print("Goal cell: ", goal_cell)
            
            xs, ys = self.get_line_indices(start_cell, goal_cell)
            
            # for i in range(len(xs)):
            #     print (xs[i],ys[i])
            #     output_img[xs[i], ys[i]] = [0,255,255]

            output_img[start_cell] = [255,0,0]
            output_img[goal_cell] = [0,255,0]
            plt.imshow(output_img)
            plt.show()
        
        return start_cell, goal_cell

def init():
    print("-----------------------")
    print("Obstacles  - White")
    print("Start Cell - Red")
    print("Goal Cell  - Green")
    print("-----------------------")


    grid=[1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
          1,0,0,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,
          1,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,1,1,1,1,1,1,
          1,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1,
          1,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,0,0,0,0,1,
          1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,
          1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,
          1,1,1,1,1,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,1,
          0,0,0,0,1,0,0,0,1,1,1,1,1,1,1,1,0,0,0,0,0,0,1,
          0,0,0,0,1,0,0,0,1,1,0,0,0,0,0,0,1,1,1,1,1,1,1,
          0,0,0,0,1,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,
          0,0,0,0,1,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,
          0,0,0,0,1,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,
          0,0,0,0,1,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,
          0,0,0,0,1,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,
          0,0,0,0,1,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,
          0,0,0,0,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0]

    grid = np.array(grid, dtype=np.uint8)
    grid.resize(17,23)
    # display_grid(grid)

    for i in range(0,10):
        pq = PlannerQuery(grid)
        pq.generate(plot=False)
        print("-----------------")

if __name__ == "__main__":
    init()