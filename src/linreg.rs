use fixed::types::I32F32;

pub fn linear_regression_and_r_squared(
    data_points: &[(I32F32, I32F32); 10],
) -> (I32F32, I32F32, I32F32) {
    let n = I32F32::from_num(data_points.len());
    let mut sum_x = I32F32::from_num(0);
    let mut sum_y = I32F32::from_num(0);
    let mut sum_xy = I32F32::from_num(0);
    let mut sum_x2 = I32F32::from_num(0);

    for &(x, y) in data_points {
        sum_x += x;
        sum_y += y;
        sum_xy += x * y;
        sum_x2 += x * x;
    }

    let mean_y = sum_y / n;
    let slope = (n * sum_xy - sum_x * sum_y) / (n * sum_x2 - sum_x * sum_x);
    let intercept = (sum_y - slope * sum_x) / n;

    // Calculate R squared
    let mut ss_tot = I32F32::from_num(0); // Total sum of squares
    let mut ss_res = I32F32::from_num(0); // Residual sum of squares

    for &(x, y) in data_points {
        let y_hat = slope * x + intercept;
        let diff_y = y - mean_y;
        let diff_y_hat = y - y_hat;
        ss_tot += diff_y * diff_y; // Squaring by multiplication
        ss_res += diff_y_hat * diff_y_hat; // Squaring by multiplication
    }

    let r_squared = I32F32::from_num(1) - (ss_res / ss_tot);

    (slope, intercept, r_squared)
}
