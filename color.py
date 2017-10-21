#!/usr/bin/env python
#Source: https://www.pyimagesearch.com/2014/08/04/opencv-python-color-detection/

import sys
import numpy as np
import math
import argparse
import cv2
from matplotlib import pyplot as plt

class color_detector:

    def __init__(self):
        #BGR Format
        self.boundaries = [
            ([200, 150, 0], [255, 200, 50]),
            ([50, 100, 25], [150, 255, 100])
        ]

    def filterColors(self, image):
        """
            So far fitlers colours out and draws centre of each face and
            its direction
            TODO: only choose best face for each domino
        :param image:
        :return:
        """
        results = []
        xPoint = 550
        yPoint = 300

        #for i in range(-10,10):
         #  for j in range(-10,10):
                #print(image.item((xPoint + i,yPoint + j,0)), end=' ')
                #print(image.item((xPoint + i, yPoint + j, 1)), end=' ')
                #print(image.item((xPoint + i, yPoint + j, 2)))
        #cv2.imshow("images", image)
        for i in range(0,3):
            print(i," ", image.item((xPoint, yPoint,i)))
        i = 0
        for (lower, upper) in self.boundaries:
            #Create numpy arrays from boundaries
            lower = np.array(lower, dtype = "uint8")
            upper = np.array(upper, dtype="uint8")
            #Find colour within the boundaries and apply mask
            mask = cv2.inRange(image, lower, upper)
            output = cv2.bitwise_and(image, image, mask=mask)
            color = cv2.medianBlur(output.copy(), 9)
            position, (xPos, yPos) = self.__getCentres(color)
            sides = self.__getConnectedComponents(color)
            print("Sides: ", len(sides))
            for side in sides:
                phi = self.__getOrientation(side)
                grad = int(math.tan(phi * math.pi / 180))
                for i in range(0,15):
                    for j in range(0, 3):
                        position.itemset((xPos + i, yPos + i*grad, j), 255)
                cv2.imshow("images", np.hstack([image,position]))
                #cv2.imwrite("filter(" + str(i) + ").jpeg", np.hstack([image,side]))
                cv2.waitKey(0)
                #i += 1
            #cv2.imshow("images", np.hstack([image, position]))
            #cv2.imwrite("position(" + str(i) + ").jpeg", np.hstack([image,position]))
            cv2.waitKey(0)

    def __getOrientation(self, image):
        """
            Gets the orientation of an image of one face of the dominoe
        :param image of one face
        :return orientation of face in degrees
        """
        size = image.shape
        #object = self.__filterAbsolute(image)
        object1 = image.copy()
        x1 = []
        y1 = []
        for i in range(0, size[0]):
            row = object1[i, :, 0]
            if (np.sum(row) > 0):
                x1.append(i)
                yCoord = int(np.average(np.where(row > 0)))
                y1.append(yCoord)
                #for j in range(0, 3):
               #     object1.itemset((i, yCoord, j), 255)
        object2 = image.copy()
        x2 = []
        y2 = []
        for i in range(0,size[1]):
            row = object2[:, i, 0]
            if(np.sum(row) > 0):
                y2.append(i)
                xCoord = int(np.average(np.where(row>0)))
                x2.append(xCoord)
              #  for j in range(0, 3):
              #      object2.itemset((xCoord, i, j), 255)
        if len(x1) < len(x2):
            minLength = len(x1)
        else:
            minLength = len(x2)
        #object3 = image.copy()
        for i in range(0, minLength):
            x1[i] = int((x1[i] + x2[-i])/2)
            y1[i] = int((y1[i] + y2[-i])/2)
            #for j in range(0, 3):
             #   object3.itemset((x1[i], y1[i], j), 255)
        grad = self.__getGradient(x1, y1)
        return math.atan(grad) * 180 / math.pi


    def __getGradient(self, x, y):
        """
            Gets gradient of set of points through linear regression
        :param xCoords
        :param yCoords
        :return gradient
        """
        x = np.array(x, dtype=int)
        y = np.array(y, dtype=int)
        xy = np.sum(x * y)
        xTot = np.sum(x)
        return (x.size * xy - xTot * np.sum(y)) / (x.size * np.sum(x * x) - xTot * xTot)





    def __getCentres(self, image):
        """
            Gets the centre of the face given by the image
        :param image of single face of dominoe
        :return image with marked centre, and coords of centre point (x,y)
        """
        size = image.shape
        color = self.__filterAbsolute(image)
        verticalTot = 0
        verticalNum = 0
        for i in range(0, size[0]):
            num = np.sum(color[i, :, 0])
            verticalNum += num
            verticalTot += num * i
        horizTot = 0
        horizNum = 0
        for i in range(0, size[1]):
            num = np.sum(color[:, i, 0])
            horizNum += num
            horizTot += num * i
        verticalTot /= verticalNum
        verticalTot = int(verticalTot)
        horizTot /= horizNum
        horizTot = int(horizTot)
        for i in range(-5, 6):
            for j in range(0, 3):
                image.itemset((verticalTot + i, horizTot, j), 255)
                image.itemset((verticalTot, horizTot + i, j), 255)
        return image, (verticalTot, horizTot)

    def __getConnectedComponents(self, image):
        """
            Finds all connected components and creates seperate image for each.
            Source: https://en.wikipedia.org/wiki/Connected-component_labeling
        :param image in a single color to analyse
        :return list of images of components
        """
        size = image.shape
        binary = self.__filterAbsolute(image)
        color = []
        linked = [np.zeros(1, dtype=np.int) for i in range(0, size[0])]
        nextLabel = 1
        labels = [[0] * size[1] for i in range(size[0])]
        for i in range(0,size[0]):
            for j in range(0, size[1]):
                if(binary.item(i, j, 0) > 0):
                    neighbours = self.__getNeighbours(i, j, binary)
                    if len(neighbours) == 0:
                        linked[nextLabel][0] = nextLabel
                        labels[i][j] = nextLabel
                        color.append(image[i,j,:])
                        nextLabel += 1
                    else :
                        l = self.__getNeighbourLabels(labels, neighbours)
                        l = l[np.nonzero(l)]
                        labels[i][j] = min(l)
                        #if(labels[i][j] < 0):
                           # print("Min: ", l)
                        for x in l:
                            linked[x] = np.unique(np.concatenate((linked[x],l)))
        uniqLabels = []
        colorOut = [[0] * size[1] for i in range(size[0])]
        for i in range(0, size[0]):
            for j in range(0, size[1]):
                if(binary.item(i, j, 0) > 0):
                    labels[i][j] = self.__findLabel(labels[i][j], linked)
                    colorOut[i][j] = color[labels[i][j] - 1]
                    if(labels[i][j] not in uniqLabels):
                        uniqLabels.append(labels[i][j])
        output = [np.zeros(shape=size, dtype="uint8") for i in range(0, len(uniqLabels))]
        for i in range(0, size[0]):
            for j in range(0, size[1]):
                if(labels[i][j] > 0):
                    np.copyto(output[uniqLabels.index(labels[i][j])][i,j], colorOut[i][j])

        return output


    def __getNeighbours(self, x, y, image):
        """
            Get all neighbouring points with the same value of given point
            8-neighbour
        :param xCoord
        :param yCoord
        :param image being analysed
        :return list of neighbouring coordinate tuples (x,y)
        """
        current = image.item(x,y, 0)
        size = image.shape
        neighbours = []
        if x > 0 and image.item(x-1, y, 0) == current: neighbours.append((x-1, y))
        if y > 0 and x > 0 and image.item(x-1,y-1, 0) == current:
            neighbours.append((x-1, y-1))
        if y > 0 and image.item(x, y-1, 0) == current: neighbours.append((x, y-1))
        if x > 0 and y+1 < size[1] and image.item(x-1, y+1, 0) == current:
            neighbours.append((x-1, y+1))
        return neighbours

    def __getNeighbourLabels(self, labels, neighbours):
        """
            Gets label of neighbouring points
        :param labels matrix containing point labels
        :param neighbours list of coordinate tuples (x,y)
        :return ndarray of neighbour's labels
        """
        nLabels = np.zeros(len(neighbours), dtype=np.int)
        for i, (x, y) in enumerate(neighbours):
            if(labels[x][y] not in nLabels):
                nLabels.itemset(i, labels[x][y])
        return nLabels

    def __findLabel(self, target, linked):
        """
            Root search to find highest parent label to given target
            Recursively searches tree
        :param target label
        :param matrix of linked labels
        :return parent label
        """
        x = min(linked[target])
        linked[target] = np.array([x])
        if (x == target):
            return x
        else:
            return self.__findLabel(x, linked)

    def __filterAbsolute(self, image):
        """
            Creates binary image of points which are not black
        :param image to filter
        :return image with 0 or 1 in first color element
        """
        size = image.shape
        result = image.copy()
        for i in range(0, size[0]):
            for j in range(0, size[1]):
                if result.item((i, j, 0)) > 0:
                    result.itemset((i, j, 0), 1)
                else:
                    result.itemset((i, j, 0), 0)
        return result

def main():
    """
        Static method for running color detection on images
    :param filename of file to analyse
    """
    if(len(sys.argv) < 2):
        print("Missing filename")
        return 0;
    cd = color_detector()
    img = cv2.imread(sys.argv[1], 1)
    cd.filterColors(img)

    return 0

if __name__ == '__main__':
    sys.exit(main())