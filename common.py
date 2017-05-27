import math

def grayscale(img):
    """Applies the Grayscale transform
    This will return an image with only one color channel
    but NOTE: to see the returned image as grayscale
    (assuming your grayscaled image is called 'gray')
    you should call plt.imshow(gray, cmap='gray')"""
    return cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
    # Or use BGR2GRAY if you read an image with cv2.imread()
    # return cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

def canny(img, low_threshold, high_threshold):
    """Applies the Canny transform"""
    return cv2.Canny(img, low_threshold, high_threshold)

def gaussian_blur(img, kernel_size):
    """Applies a Gaussian Noise kernel"""
    return cv2.GaussianBlur(img, (kernel_size, kernel_size), 0)

def region_of_interest(img, vertices):
    """
    Applies an image mask.

    Only keeps the region of the image defined by the polygon
    formed from `vertices`. The rest of the image is set to black.
    """
    #defining a blank mask to start with
    mask = np.zeros_like(img)

    #defining a 3 channel or 1 channel color to fill the mask with depending on the input image
    if len(img.shape) > 2:
        channel_count = img.shape[2]  # i.e. 3 or 4 depending on your image
        ignore_mask_color = (255,) * channel_count
    else:
        ignore_mask_color = 255

    #filling pixels inside the polygon defined by "vertices" with the fill color
    cv2.fillPoly(mask, vertices, ignore_mask_color)

    #returning the image only where mask pixels are nonzero
    masked_image = cv2.bitwise_and(img, mask)
    return masked_image


def draw_lines(img, lines, color=[255, 0, 0], thickness=2):
    """
    NOTE: this is the function you might want to use as a starting point once you want to
    average/extrapolate the line segments you detect to map out the full
    extent of the lane (going from the result shown in raw-lines-example.mp4
    to that shown in P1_example.mp4).

    Think about things like separating line segments by their
    slope ((y2-y1)/(x2-x1)) to decide which segments are part of the left
    line vs. the right line.  Then, you can average the position of each of
    the lines and extrapolate to the top and bottom of the lane.

    This function draws `lines` with `color` and `thickness`.
    Lines are drawn on the image inplace (mutates the image).
    If you want to make the lines semi-transparent, think about combining
    this function with the weighted_img() function below
    """
    for line in lines:
        for x1,y1,x2,y2 in line:
            cv2.line(img, (x1, y1), (x2, y2), color, thickness)

# 線分の拡張と平滑化を行う。(平滑化は未実装)
def draw_ext_LiRe_lines(img, lines, color=[0, 255, 0], thickness=2):

    d = 300 # required extend length

    line_left_x, line_left_y, line_right_x, line_right_y

    for line in lines:
        for x1,y1,x2,y2 in line:
            if (x2 != x1):
                slope = (y2-y1)/(x2-x1)
                sita = np.arctan(slope)
                if (slope > 0): # 傾きに応じて場合分け
                        lines_left_x.extend([x1,x2])
                        lines_left_y.extend([y1,y2])
                elif (slope < 0):
                        lines_right_x.extend([x1,x2])
                        lines_right_y.extend([y1,y2])

    f_pf_left = np.poly1d(np.polyfit(lines_left_x.extend,lines_left_y.extend,1))
    f_pf_right = np.poly1d(np.polyfit(lines_right_x.extend,lines_right_y.extend,1))

    cv2.line(img, (np.min(lines_left_x), int(f_pf_left(np.min(lines_left_x)))), (int(img.shape[1]/2)-5, int(f_pf_left(int(img.shape[1]/2)-5))), color, thickness)
    cv2.line(img, (np.min(lines_right_x), int(f_pf_right(np.min(lines_right_x)))), (int(img.shape[1]/2)+5, int(f_pf_right(int(img.shape[1]/2)+5))), color, thickness)


# 線分の延長を行う。.Append、や　np.poly1d、　np.polyfit(x,y,n)を利用した効率化が必要
# np.polyfit(x,y,n): n次式で２変数x,yの回帰分析
def draw_ext_lines(img, lines, color=[255, 0, 0], thickness=2):
    d = 300 # required extend length
    for line in lines:
        for x1,y1,x2,y2 in line:
            if (x2 != x1):
                slope = (y2-y1)/(x2-x1)
                sita = np.arctan(slope)
                if (slope > 0): # 傾きに応じて場合分け
                    if (x2 > x1):
                        x3 = int(x2 + d*np.cos(sita))
                        y3 = int(y2 + d*np.sin(sita))
                        cv2.line(img, (x3, y3), (x1, y1), color, thickness)
                    else:
                        x3 = int(x1 + d*np.cos(sita))
                        y3 = int(y1 + d*np.sin(sita))
                        cv2.line(img, (x3, y3), (x2, y2), color, thickness)
                elif (slope < 0):
                    if (x2 > x1):
                        x3 = int(x1 - d*np.cos(sita))
                        y3 = int(y1 - d*np.sin(sita))
                        cv2.line(img, (x3, y3), (x2, y2), color, thickness)
                    else:
                        x3 = int(x2 - d*np.cos(sita))
                        y3 = int(y2 - d*np.sin(sita))
                        cv2.line(img, (x3, y3), (x1, y1), color, thickness)

def hough_lines(img, rho, theta, threshold, min_line_len, max_line_gap):
    """
    `img` should be the output of a Canny transform.

    Returns an image with hough lines drawn.
    """
    lines = cv2.HoughLinesP(img, rho, theta, threshold, np.array([]), minLineLength=min_line_len, maxLineGap=max_line_gap)
    line_img = np.zeros((img.shape[0], img.shape[1], 3), dtype=np.uint8)
    ###draw_lines(line_img, lines)
    if (not isinstance(lines,type(None))):
        draw_ext_lines(line_img, lines)

    return line_img

# Python 3 has support for cool math symbols.

def weighted_img(img, initial_img, α=0.8, β=1., λ=0.):
    """
    `img` is the output of the hough_lines(), An image with lines drawn on it.
    Should be a blank image (all black) with lines drawn on it.

    `initial_img` should be the image before any processing.

    The result image is computed as follows:

    initial_img * α + img * β + λ
    NOTE: initial_img and img must be the same shape!
    """
    return cv2.addWeighted(initial_img, α, img, β, λ)
