#!/usr/bin/env python
#Source: https://www.pyimagesearch.com/2014/08/04/opencv-python-color-detection/

import sys
import numpy as np
import math
import heapq
import cv2
import edge



class color_detector:

    def __init__(self):
        #BGR Format
        self.boundaries = [
           # ([0, 0, 0], [10, 10, 10]),  # Black
           # ([200, 150, 0], [255, 200, 50]),    #Blue
           # ([50, 100, 25], [150, 255, 100])    #Green

           #([50,50,30], [255, 100, 50]),       #Blue Lab
           #([30, 30, 20], [50, 50, 40])        #Green Lab
            #HSI Format
            ([95, 110, 20], [120, 255, 200]), #Blue
            ([85, 50, 17], [105, 150,30])    #Green
            #HSI Dark
            #([10, 110, 30], [40, 140, 70])  #Blue


        ]
        self.ed = edge.edge_detector()

    def filterColors(self, image):
        """
            Finds each face of all dominoes in image
        :param image to analyse
        :return list of images of each domino face
        """
        results = []
        #size = image.shape
        #print "Size: ", size[0], " ", size[1]
        #xPoint = 200
        #yPoint = 350
        #hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        #for i in range(-5,5):
        #    for j in range(-5,5):
        #        print image.item((xPoint + i,yPoint + j,0)),
        #        print image.item((xPoint + i, yPoint + j, 1)),
        #        print image.item((xPoint + i, yPoint + j, 2))
        #        image[xPoint + i, yPoint + j, :] = [255, 255, 255]
        #cv2.imshow("images", image)
        #cv2.waitKey(0)
        colors = self.__getColors(image)
        sides = []
        #print "Displaying Colours: (", len(colors), ")"
        for color in colors:
            #cv2.imshow("images", np.hstack([image,color]))
            #cv2.waitKey(0)
            sides.extend(self.__getConnectedComponents(color))
        #print "Showing Faces.."
        #for side in sides:
        #    cv2.imshow("images", np.hstack([image, side]))
        #    cv2.waitKey(0)
        return sides

    def getAllDominoes(self, image):
        """
            Finds all dominoes in the image and finds their position and
            orientation ordered by face size
        :param image from camera
        :return list of tuples (xPos, yPos, orientation, locationImage)
        """

        faces = self.filterColors(image)
        faceQueue = []
        results = []
        for face in faces:
            xPos, yPos = self.__getCentres(face)
            phi = self.__getOrientation(face)
            location = self.drawLocation(face, (xPos, yPos), phi)
            faceSize = self.__getFaceSize(face)
            heapq.heappush(faceQueue, (-faceSize, (xPos, yPos, phi, location)))
        outImage = image.copy()
        for i in range(len(faceQueue)):
            results.append(heapq.heappop(faceQueue)[1])
            outImage = self.drawLocation(outImage, (results[i][0], results[i][1]), results[i][2])
        #print "All domino positions,.."
        #cv2.imshow("images", outImage)
        #cv2.waitKey(0)
        #print("Faces found: ", len(results))
        return results


    def getNextDomino(self, image):
        """
            Gets domino with largest face (closest) from image. Returns empty
            tuple if no faces are found
        :param image from camera
        :return tuple (xPos, yPos, orientation, locationImage)
        """
        faces = self.filterColors(image)
        max = 0
        maxDomino = -1
        for i, face in enumerate(faces):
            faceSize = self.__getFaceSize(face)
            if faceSize > max:
                max = faceSize
                maxDomino = i
        if maxDomino > -1:
            xPos, yPos = self.__getCentres(faces[maxDomino])
            phi = self.__getOrientation(faces[maxDomino])
            locationImage = self.drawLocation(faces[maxDomino].copy(), (xPos, yPos), phi)
            #print "Next Domino"
            #cv2.imshow("images", np.hstack([image,locationImage]))
            #cv2.waitKey(0)
            return (xPos, yPos, phi, faces[maxDomino])
        return ()

    def toNextDomino(self, image):
        """
            Gets position of next domino relative to origin. Returns empty
            tuple if no faces are found
        :param image from camera
        :return tuple (relativeDistance, relativeAngle, locationImage)
        """
        nextDomino = self.getNextDomino(image)
        if nextDomino:
            dominoLength, lengthOutput = self.getLongEdgeLength(nextDomino, nextDomino[3])
            #print(dominoLength)
            #cv2.imshow("images", np.hstack([nextDomino[3], lengthOutput]))
            #cv2.waitKey(0)
            xDom = nextDomino[0] + dominoLength/4 * math.cos(nextDomino[2] * math.pi / 180)
            yDom = nextDomino[1] + dominoLength / 4 * math.sin(nextDomino[2] * math.pi / 180)
            nextDomino = (xDom, yDom, nextDomino[2])
            relDist, relAngle, origin, originImage = self.getPosRelative(image, nextDomino)
            if relDist > -1:
                relOrientation = nextDomino[2] - origin[2]
                outImage = self.drawPoint(image.copy(), (nextDomino[0], nextDomino[1]), size=10)
                self.drawLocation(outImage, (origin[0], origin[1]), origin[2])
                self.drawLocation(outImage, (origin[0], origin[1]) , origin[2] + relAngle)
                pixelDist, distImage = self.getLongEdgeLength(origin, originImage)
                relDist *= 83/(pixelDist/2)
                return (relDist, relAngle, relOrientation, outImage)
        return ()

    def getLongEdgeLength(self, faceTuple, image):
        """
            Gets the size of the longest edge of a given face
        :param faceTuple (xPos, yPos, angle, Image)
        :return edge distance and image of distance measured
        """
        grad = math.tan(faceTuple[2] * math.pi / 180)
        #print(image)
        output = image.copy()
        size = image.shape
        max = (0,0)
        i = 0
        for x in range(faceTuple[0], size[0]):
            y = faceTuple[1] + i * grad
            if abs(int(y)) < size[1] and image[x, int(y), 0] > 0:
                max = (x,y)
                output[x,int(y),:] = [255,255,255]
                #output[2 * faceTuple[0] - x, int(2 * faceTuple[1] - y), :] = [255,255,255]
            i += 1
        dist = math.sqrt(pow(float(max[0]-faceTuple[0]),2) + pow(float(max[1]-faceTuple[1]),2))
        return dist * 2 , output

    def getOrigin(self, image):
        """
            Finds position and orientation of origin represented by
            black cross from image.
        :param image from camera
        :return tuple (xPos, yPos, orientation)
        """
        output = self.__getBlack(image)
        #print "Skeleton..."
        #cv2.imshow("images", np.hstack([image, output]))
        #cv2.waitKey(0)
        output = self.__getOriginOnly(output)
        #cv2.imshow("images", np.hstack([image, output]))
        #cv2.waitKey(0)
        skel = self.__blur(self.__getSkeleton(output), 3)
        #cv2.imshow("images", np.hstack([image, skel]))
        #cv2.waitKey(0)
        xtremes = self.__getExtremePoints(skel)
        xPos, yPos = self.__getIntersection(xtremes, output)
        print xPos, yPos
        if xPos > -1:
            orientation = self.__getOrientationPoints(xPos, yPos, xtremes, image.copy())
            originImage = self.drawLocation(image.copy(), (xPos, yPos), orientation)
            #print "Origin..", orientation
            #cv2.imshow("images", np.hstack([skel, originImage]))
            # cv2.waitKey(0)
            return (xPos, yPos, orientation), output
        return (), -1

    def __getBlack(self, image):
        invImage = cv2.bitwise_not(image.copy())
        hsv = cv2.cvtColor(invImage, cv2.COLOR_BGR2HSV)
        lowerBound = np.array([0, 0, 235], dtype="uint8")
        upperBound = np.array([180, 20, 255], dtype="uint8")
        mask = cv2.inRange(hsv, lowerBound, upperBound)
        output = cv2.bitwise_and(invImage, invImage, mask=mask)
        output = cv2.medianBlur(output, 9)
        #cv2.imshow("images", np.hstack([image, output]))
        #cv2.waitKey(0)
        return output


    def drawLocation(self, image, center, angle):
        """
            Draw location of dominoe on image given position and orientation
        :param image to draw on:
        :param center position of domino (x,y):
        :param angle of domino in degrees:
        :return image with position drawn in white:
        """
        size = image.shape
        image = self.drawPoint(image, (center[0], center[1]))
        #print angle
        x = center[0] + 30 * math.cos(angle * math.pi / 180.0)
        y = center[1] + 30 * math.sin(angle * math.pi / 180.0)
        #x = center[0] + 30 * math.sin((angle + 180) * math.pi / 180)
        #y = center[1] + 30 * math.cos((angle + 180) * math.pi / 180)
        cv2.line(image, (center[1], center[0]), (int(y), int(x)), (255,255,255))
        return image

    def drawPoint(self, image, point, size=20):
        """
            Draws given point on image using a white cross
        :param image to draw on
        :param point to draw
        :return same image with point drawn on
        """
        imSize = image.shape
        point = (int(point[0]), int(point[1]))
        for i in range(-int(size/2), int(size/2)+1):
            if point[0] + i < imSize[0] and point[1] + i < imSize[1]:
                image[point[0] + i, point[1], :] = [255,255,255]
                image[point[0], point[1] + i, :] = [255,255,255]
        return image

    def getPosRelative(self, image, position):
        """
            Gets relative position of given domino position and orientation
            with origin found in image
        :param image from camera
        :param position of domino (xPos, yPos, orientation)
        :return dist - relative distance to origin
        :return angle - relative from origin angle
        :return originPos - tuple (xPos, yPos, orientation)
        """

        originPos, originImage = self.getOrigin(image)
        if originPos:
            dx = position[0] - originPos[0]
            dy = position[1] - originPos[1]
            dist = math.sqrt(pow(dx,2) + pow(dy,2))
            angle = math.atan2(dy, dx) * 180 / math.pi
            return dist, angle - originPos[2], originPos, originImage
        return -1, -1, -1, -1

    def __getOrientationPoints(self, xPos, yPos, points, image):
        """
            Gets orientation of origin given extreme points
        :param xPos of centre of mass
        :param yPos of centre of mass
        :param points extremities of origin
        :return angle of origin in degrees
        """
        max = 0
        maxPoint = 0
        for i, (x, y) in enumerate(points):
            dist = math.sqrt(pow(xPos-x, 2) + pow(yPos-y, 2))
            if dist > max:
                max = dist
                maxPoint = i
        #print "Xtreme", points[maxPoint]
        #cv2.imshow("images", self.drawPoint(image, points[maxPoint]))
        #cv2.waitKey(0)
        return math.atan2(points[maxPoint][1] - yPos,points[maxPoint][0] - xPos) * 180 / math.pi

    def __getIntersection(self, points, image):
        """
            Finds point where cross intersects given by line endpoints.
            Ie center of origin
        :param points list of 4 points (x,y) of line endpoints
        :return: x, y coordinates of intersection point
        """
        if points:
            points = self.__reversePoints(points)
            if len(points) == 4:
                cv2.line(image, points[0], points[1], (255,255,255))
                cv2.line(image, points[2], points[3], (255, 255, 255))
                #cv2.imshow("images", image)
                #cv2.waitKey(0)
                x1 = points[0][0]
                y1 = points[0][1]
                m1 = float(points[1][1] - points[0][1])/float(points[1][0] - points[0][0])
                x2 = points[2][0]
                y2 = points[2][1]
                m2 = float(points[3][1] - points[2][1])/float(points[3][0] - points[2][0])
                if m1 != m2:
                    xPos = (y2 - y1 + m1 * x1 - m2 * x2)/(m1-m2)
                    yPos = m1 * (xPos -x1) + y1
                    return int(yPos), int(xPos)
        return -1, -1

    def __reversePoints(self, points):
        """
            Reverses orderering of a list of tuples
        :param points list of tuples
        :return list of reversed tuples
        """
        output = []
        for p in points:
            if p:
                output.append(((p[1]), p[0]))
        return output

    def __getExtremePoints(self, image):
        """
            Gets extremeties of given image of origin (skeletonised)
        :param BGR image to analyse
        :return list of 4 endpoints of lines
        """
        imOut = image.copy()
        result = []
        size = image.shape
        #cv2.imshow("images", image)
        #cv2.waitKey(0)
        first = ()
        last = ()
        half = image.copy()
        for i in range(size[0]):
            for j in range(size[1]):
                if image[i,j,0] > 0:
                    if not first: first = (i,j)
                    else: last = (i, j)
        if first and last:
            cv2.line(imOut, (first[1], first[0]), (last[1], last[0]), (0,0,0), thickness=3)
            #print "Removed"
            half = self.drawPoint(imOut.copy(), first)
            half = self.drawPoint(half, last)
            #cv2.imshow("images", half)
            #cv2.waitKey(0)
            result.append(first)
            result.append(last)
            first = ()
            for j in range(size[1]):
                for i in range(size[0]):
                    if imOut[i,j,0] > 0:
                        if not first: first =  (i,j)
                        else: last = (i,j)
            result.append(first)
            result.append(last)
            print "extremes"
            xPoints = image.copy()
            for p in result:
                if p:
                    self.drawPoint(xPoints, p)
            #cv2.imshow("images",np.hstack([half, xPoints]) )
            #cv2.waitKey(0)
        return result

    def __blur(self, image, num):
        """
            Removes points with less than num neighbours
        :param image to analyse
        :param num min number of neighbours
        :returnm image with points removed
        """
        output = image.copy()
        size = image.shape
        for i in range(size[0]):
            for j in range(size[1]):
                if image[i,j,0] > 0:
                    neighbours  = 0
                    for m in range(i-1,i+2):
                        for n in range(j-1, j+2):
                            if m < size[0] and n < size[1] and \
                                    (not (m == i and n == j)) and image[m,n,0] > 0:
                                neighbours += 1
                    if neighbours < num:
                        output[i,j,:] = [0,0,0]
        return output

    def __getSkeleton(self, image):
        """
            Performs skeletonisation of given BGR image
            Source: http://opencvpython.blogspot.com.au/2012/05/skeletonization-using-opencv-python.html
        :param image to analyse
        :return BGR skeleton of image
        """
        image = cv2.medianBlur(image, 9)
        gray = cv2.cvtColor(image.copy(), cv2.COLOR_BGR2GRAY)
        ret, gray = cv2.threshold(gray, 100, 255, cv2.THRESH_BINARY)
        size = np.size(gray)
        #cv2.imshow("images", gray)
        skel = np.zeros(gray.shape, dtype="uint8")
        #cv2.waitKey(0)
        element =  cv2.getStructuringElement(cv2.MORPH_CROSS, (3,3))
        done = False
        while not done:
            eroded = cv2.erode(gray, element)
            temp = cv2.dilate(eroded, element)
            temp = cv2.subtract(gray, temp)
            skel = cv2.bitwise_or(skel, temp)
            gray = eroded.copy()

            zeros = size - cv2.countNonZero(gray)
            if zeros == size:
                done = True
        output = np.zeros(image.shape, dtype="uint8")
        size = output.shape
        for i in range(size[0]):
            for j in range(size[1]):
                if skel[i,j] > 0:
                    output[i,j,:] = [255,255,255]
        return output


    def __getOriginOnly(self, image):
        """
            Removes all components that are not the origin in the image
        :param image to edit
        :return image with manipulators whited out
        """
        size = image.shape
        output = image.copy()
        compSizes = []
        cMax = 0
        components = self.__getConnectedComponents(image)
        for cImage in components:
            #isManipulator = False
            #for i in range(size[1]):
            #    if cImage[0,i,0] > 0:
            #        output -= cImage
            #        isManipulator = True
            #        break
            #if not isManipulator:
            cSize = np.sum(cImage)
            if cSize > cMax:
                cMax = cSize
                output = cImage
        return output

    def __getFaceSize(self, face):
        """
            Gets the size of the domino face given a filtered image of it
        :param face image
        :return size of face in number of pixels:
        """
        face = self.__filterAbsolute(face)
        factor = 1
        colorIndex = 0
        for i, (lower, upper) in enumerate(self.boundaries):
            # Create numpy arrays from boundaries
            lower = np.array(lower, dtype="uint8")
            upper = np.array(upper, dtype="uint8")
            # Find colour within the boundaries and apply mask
            mask = cv2.inRange(face, lower, upper)
            output = cv2.bitwise_and(face, face, mask=mask)
            if np.sum(output) > 0:
                colorIndex  = i
        if colorIndex == 1:
            factor = 2.4
        return np.sum(face) * factor



    def __getColors(self, image):
        """
            Filters image getting blue and green colors and making seperate images
        :param image to analyse:
        :return list of images seperated by color:
        """
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        result = []
        for (lower, upper) in self.boundaries:
            # Create numpy arrays from boundaries
            lower = np.array(lower, dtype="uint8")
            upper = np.array(upper, dtype="uint8")
            # Find colour within the boundaries and apply mask
            mask = cv2.inRange(hsv, lower, upper)
            output = cv2.bitwise_and(image, image, mask=mask)
            result.append(cv2.medianBlur(output, 9))
        return result

    def __getOrientation(self, image):
        """
            Gets the orientation of an image of one face of the dominoe
        :param image of one face
        :return orientation of face in degrees
        """
        size = image.shape
        x = []
        y = []
        output = image.copy()

        for i in range(size[0]):
            for j in range(size[1]):
                if image[i,j,0] > 0:
                    x.append(i)
                    y.append(j)
                    output[i,j,:] = [255,255,255]
        if x and y:
            grad = self.__getGradient(x, y)
            #cv2.imshow("images", np.hstack([image, output]))
            #cv2.waitKey(0)
            if grad == float("inf"):
                return 90
            return math.atan(grad) * 180 / math.pi
        return 0

    def __isPerpendicular(self, grad1, grad2):
        """
            Checks to see whether two gradients are perpendicular
        :param grad1:
        :param grad2:
        :return:
        """
        angle1 = math.atan(grad1) * 180 / math.pi
        angle2 = math.atan(grad2) * 180 / math.pi
        diff = int(angle1 - angle2) % 90
        return abs(diff) < 2

    def __getRange(self, x, y):
        """
            Gets range of x & y coordinates
        :param x:
        :param y:
        :return:
        """
        return math.sqrt(pow(x[-1] - x[0], 2) + pow(y[-1] - y[0], 2))

    def __getGradient(self, x, y):
        """
            Gets gradient of set of points through linear regression
        :param xCoords
        :param yCoords
        :return gradient
        """
        x = np.array(x, dtype=float)
        y = np.array(y, dtype=float)
        xy = np.sum(x * y)
        xTot = np.sum(x)
        bot = x.size * np.sum(x * x) - xTot * xTot
        if bot == 0:
            return float("inf")
        return (x.size * xy - xTot * np.sum(y))/bot


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
        return verticalTot, horizTot

    def __getConnectedComponents(self, image):
        """
            Finds all connected components and creates seperate image for each.
            Uses two-pass method
            Source: https://en.wikipedia.org/wiki/Connected-component_labeling
        :param image in a single color to analyse
        :return list of images of components
        """
        size = image.shape
        binary = self.__filterAbsolute(image)
        linked = [[0]]
        nextLabel = 1
        labels = [[0] * size[1] for i in range(size[0])]
        for i in range(size[0]):
            if np.sum(binary[i,:,0] > 0):
                for j in range(size[1]):
                    if(binary.item(i, j, 0) > 0):
                        neighbours, nLabels, = self.__getNeighbours(i, j, labels, binary)
                        if len(neighbours) == 0:
                            linked.append([nextLabel])
                            labels[i][j] = nextLabel
                            nextLabel += 1
                        elif len(nLabels) > 0:
                            labels[i][j] = min(nLabels)
                            for x in nLabels:
                                linked[x] = list(set().union(linked[x], nLabels))
        uniqLabels = []
        output = []
        for i in range(size[0]):
            if np.sum(binary[i,:,0] > 0):
                for j in range(size[1]):
                    if(binary.item(i, j, 0) > 0):
                        lab = labels[i][j]
                        labels[i][j] = self.__findLabel(lab, linked)
                        linked[lab] = [labels[i][j]]
                        if(labels[i][j] not in uniqLabels):
                            uniqLabels.append(labels[i][j])
                            imageOut = np.zeros(shape=size, dtype="uint8")
                            output.append(imageOut)
                        output[uniqLabels.index(labels[i][j])][i, j, :] = image[i, j, :]
        return output


    def __getNeighbours(self, x, y, labels, image):
        """
            Get all neighbouring points with the same value of given point
            8-neighbour
        :param xCoord
        :param yCoord
        :param image being analysed
        :return list of neighbouring coordinate tuples (x,y)
        """
        size = image.shape
        neighbours = []
        nLabels = []
        if x > 0 and image.item(x-1, y, 0) > 0:
            neighbours.append((x-1, y))
            nLabels.append(labels[x-1][y])
        if y > 0 and x > 0 and image.item(x-1,y-1, 0) > 0:
            neighbours.append((x-1, y-1))
            nLabels.append(labels[x-1][y-1])
        if y > 0 and image.item(x, y-1, 0) > 0:
            neighbours.append((x, y-1))
            nLabels.append(labels[x][y-1])
        if x > 0 and y+1 < size[1] and image.item(x-1, y+1, 0) > 0:
            neighbours.append((x-1, y+1))
            nLabels.append(labels[x-1][y+1])
        nLabels = filter(lambda a: a != 0, nLabels)
        return neighbours, nLabels

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
        return (image > 0).astype(int)

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

    img = cv2.resize(img, (640, 400), interpolation=cv2.INTER_CUBIC)
    #faces = cd.filterColors(img)
    xPos, yPos, phi = cd.getOrigin(img)
    originImage = cd.drawLocation(img.copy(),(xPos, yPos), phi)
    cv2.imshow("images", np.hstack([img, originImage]))
    cv2.waitKey(0)
    return 0

if __name__ == '__main__':
    sys.exit(main())