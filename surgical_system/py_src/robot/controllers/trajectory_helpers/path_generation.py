import numpy as np

class Path_Gen: 
    @staticmethod
    def generateRaster(startPoint: np.ndarray, endPoint: np.ndarray, numPasses: int):
        
        if startPoint.size != 2 and endPoint.size !=2:
            raise ValueError("startPoint and endPoint must be size 2")
        
        xPoints = np.array([startPoint[0], endPoint[1]] * (numPasses + 2))
        yRange = np.linspace(startPoint[0], endPoint[1], numPasses+2)
        yPoints = np.repeat(yRange, 2)
        
        return np.vstack((xPoints, yPoints)).transpose() 
    
    def generatePolygon():
        pass
    
    def generateCircle(center: np.array, radius: float):
        pass
    
    
     
    

# startPoint = np.array([-5, 0])
# endPoint = np.array([5, 5])
# numPasses = 3
# xPoints = np.array([startPoint[0], endPoint[1]] * (numPasses + 2))
# yRange = np.linspace(startPoint[0], endPoint[1], numPasses+2)
# yPoints = np.array([y for y in yRange for _ in range(2)])

# points = np.vstack((xPoints, yPoints)).transpose()
# print(points)

# points = np.array([[0, 0],
#                    [1, 0],
#                    [3, 0],
#                    [0, 0]])
# distances = np.linalg.norm(points[1:] - points[:-1], axis=1)
# print(points[1:] - points[:-1])
# print(distances)
# sumDistance = np.sum(distances)

# durations = (distances / np.sum(distances)) * 10
# print(durations)