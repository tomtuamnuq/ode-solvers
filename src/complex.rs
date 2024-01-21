#[cfg(test)]
mod tests {
    use crate::rk4::Rk4;
    use crate::{OVector, System, Vector2};
    use nalgebra::{allocator::Allocator, DefaultAllocator, Dim};

    struct Test1 {}
    impl<D: Dim> System<f64, OVector<f64, D>> for Test1
    where
        DefaultAllocator: Allocator<f64, D>,
    {
        fn system(&self, x: f64, y: &OVector<f64, D>, dy: &mut OVector<f64, D>) {
            // dy/dt = iy with y = a + bi => dy/dt = ai - b = -b + ai
            dy[0] = -y[1]; // -b
            dy[1] = y[0]; // a
        }
    }

    #[test]
    fn test_complex_test1_svector() {
        let system = Test1 {};
        let t_end: f64 = 0.2;
        let real_end_state = Vector2::new(t_end.cos(), t_end.sin());
        let mut stepper = Rk4::new(system, 0., Vector2::new(1., 0.), t_end, 0.1);
        let _ = stepper.integrate();
        let x_out = stepper.x_out();
        let y_out = stepper.y_out();
        assert!(
            (*x_out.last().unwrap() - t_end).abs() < 1.0E-8,
            "x_out must end with x_end"
        );
        assert!(
            y_out
                .last()
                .unwrap()
                .relative_eq(&real_end_state, 1.0E-5, 1.0E-5),
            "The last state must be the solution at x_end!"
        );
    }
}
