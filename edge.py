#!/usr/bin/env python
import cv2
import sys
import numpy as np
from matplotlib import pyplot as plt



class edge_detector:
    def __init__(self):
        self.edges = 0

    def getEdges(self, img):
        """
            Uses Sobel filter to find edges of image
        :param img to analyse
        :return edge intensity image
        """
        gray = cv2.cvtColor (img, cv2.COLOR_BGR2GRAY)
        sobelx = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=5)
        sobely = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=5)
        combined = cv2.add(np.absolute(sobelx), np.absolute(sobely))
        return combined

    def getEdgesF(self, img, low, high):
        """
            Uses Sobel filter to find edges of image
        :param img message to analyse
        :param low value threshold
        :param high value threshold
        :return binary image
        """
        combined = self.getEdges(img)
        return self.filter(combined, low, high)

    def max2(self, matrix):
        """
            Finds maximum of matrix (image)
        :param matrix:
        :return maximum value
        """
        matrixMax = 0;
        for i in range(0, len(matrix)):
            rowMax = max(matrix[i])
            if (rowMax > matrixMax): matrixMax = rowMax
        return matrixMax

    def filter(self, matrix, lower, upper):
        """
            Filters out pixels outide the lower and upper range
        :param matrix to filter
        :param lower bound, pixels below are set to 0
        :param upper bound, pixels above are set to 0
        :return binary image matrix
        """

        rows = len(matrix)
        cols = len(matrix[0])
        result = [[0] * cols for i in range(rows)]
        for i in range(0, rows):
            for j in range(0, cols):
                if (matrix[i][j] < lower or matrix[i][j] > upper):
                    result[i][j] = 0
                else:
                    result[i][j] = 1
        return np.asarray(result)

    def mergeEdges(self, image, edges):
        """
            Merges binary edge image with color image
        :param image
        :param edges:
        :return colour image with edges blacked out
        """
        size = edges.shape
        image_out = image.copy()
        for i in range(size[0]):
            for j in range(size[1]):
                if edges[i][j] > 0:
                    image_out[i,j,:] = [255,255,255]
        return cv2.medianBlur(image_out, 9)

def main():
    """
        Static method for running edge detection on images
    :param filename of file to analyse
    """
    if(len(sys.argv) < 2):
        print("Missing filename")
        return 0;
    ed = edge_detector()
    img = cv2.imread(sys.argv[1], 0)
    #laplacian = cv2.Laplacian(img,cv2.CV_64F)


    plt.subplot(221), plt.imshow(img, cmap= 'gray')
    plt.title('Original Image'), plt.xticks([]), plt.yticks([])
    plt.subplot(222), plt.imshow(ed.getEdges(img), cmap='gray')
    plt.title('Combined Image'), plt.xticks([]), plt.yticks([])
    plt.subplot(223), plt.imshow(ed.getEdgesF(img, 1000, 5000), cmap='binary')
    plt.title('Binary 1-5'), plt.xticks([]), plt.yticks([])
    plt.subplot(224), plt.imshow(ed.getEdgesF(img, 2000, 7000), cmap='binary')
    plt.title('Binary 2'), plt.xticks([]), plt.yticks([])

    plt.show()
    return 0

if __name__ == '__main__':
    sys.exit(main())