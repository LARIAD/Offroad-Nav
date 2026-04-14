import cupy as cp

def hough_line_transform(image, theta_res=1, rho_res=1):
    # Define the range of theta and rho
    thetas = cp.deg2rad(cp.arange(-90, 90, theta_res))
    width, height = image.shape
    diag_len = cp.ceil(cp.sqrt(width * width + height * height))  # maximum rho
    rhos = cp.linspace(-diag_len, diag_len, cp.ceil(2 * diag_len / rho_res))

    # Initialize accumulator array
    accumulator = cp.zeros((len(rhos), len(thetas)), dtype=cp.uint64)

    # Indices of the non-zero pixels in the binary image
    y_idxs, x_idxs = cp.nonzero(image)

    # Vote in the accumulator array
    for i in range(len(x_idxs)):
        x = x_idxs[i]
        y = y_idxs[i]

        for j in range(len(thetas)):
            rho = cp.round(x * cp.cos(thetas[j]) + y * cp.sin(thetas[j]))
            rho_idx = cp.argmin(cp.abs(rhos - rho))
            accumulator[rho_idx, j] += 1

    return accumulator, thetas, rhos