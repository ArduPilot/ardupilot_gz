import rclpy
from rclpy.node import Node

from sgp_ipp.utils.tsp import run_tsp
from sgp_ipp.utils.sensor_placement import *
from sgp_ipp.models.osgpr import OSGPR_VFE
from sgp_ipp.models.transformations import FixedInducingTransformer

import gpflow


class OnlineIPP(Node):
    """Class to create an online IPP mission."""

    def __init__(self, X_train, num_placements=20, num_param_inducing=40):
        super().__init__('online_ipp')

        self.num_placements = num_placements

        # Initialize random SGP parameters
        likelihood_variance = 1e-4
        kernel = gpflow.kernels.RBF(variance=1.0, 
                                    lengthscales=1.0)

        # Get the initial IPP solution
        self.transformer = FixedInducingTransformer()
        self.IPP_model, _ = get_aug_sgp_sol(num_placements, 
                                            X_train,
                                            likelihood_variance,
                                            kernel,
                                            self.transformer)

        # Reorder the inducing points to match the tsp solution
        self.sgp_sol_sp = self.IPP_model.inducing_variable.Z.numpy()
        self.sgp_sol_sp = self.sgp_sol_sp[run_tsp(self.sgp_sol_sp)[0]]
        self.IPP_model.inducing_variable.Z.assign(self.sgp_sol_sp)

        # Initilize a SGPR model with random parameters for the OSGPR
        # The X_train and y_train are not used to optimize the kernel parameters
        # but they effect the initial inducing point locations, i.e., limits them
        # to the bounds of the data
        Z_init = get_inducing_pts(X_train, num_param_inducing)
        init_param = gpflow.models.SGPR((X_train, np.zeros((X_train.shape[0], 1))),
                                        kernel=kernel,
                                        inducing_variable=Z_init, 
                                        noise_variance=likelihood_variance)
        
        # Initialize the OSGPR model using the parameters from the SGPR model
        # The X_train and y_train here will be overwritten in the online phase
        X_train = np.array([[0, 0], [0, 0]])
        y_train = np.array([0, 0]).reshape(-1, 1)
        Zopt = init_param.inducing_variable.Z.numpy()
        mu, Su = init_param.predict_f(Zopt, full_cov=True)
        Kaa = init_param.kernel(Zopt)
        self.online_param = OSGPR_VFE((X_train, y_train),
                                       init_param.kernel,
                                       mu, Su[0], Kaa,
                                       Zopt, Zopt)
        self.online_param.likelihood.variance.assign(init_param.likelihood.variance)

        del init_param

        # Create a timer to publish control commands
        self.timer = self.create_timer(10, self.timer_callback)
        self.time_step = 0

        self.get_logger().info('Online IPP mission initialized')

    def ipp_update(self, time_step):
        """Update the IPP solution."""

        # Freeze the visited inducing points
        Xu_visited = self.sgp_sol_sp.copy()[:time_step]
        self.transformer.update(Xu_visited)

        # Get the new inducing points for the path
        self.IPP_model.update(self.online_param.likelihood.variance,
                              self.online_param.kernel)
        optimize_model(self.IPP_model, num_steps=100, 
                       kernel_grad=False, 
                       lr=1e-2, opt='adam')

        self.sgp_sol_sp = self.IPP_model.inducing_variable.Z
        self.sgp_sol_sp = self.IPP_model.transformer.expand(self.sgp_sol_sp).numpy()

    def param_update(self, X_new, y_new):
        """Update the OSGPR parameters."""

        # Get the new inducing points for the path
        self.online_param.update((X_new, y_new))
        optimize_model(self.online_param, opt='scipy')

    def timer_callback(self):
        """Callback function for the timer."""

        # Update the OSGPR parameters with new data
        X_new = np.random.uniform(0, 10, (10, 2))
        y_new = np.random.uniform(0, 10, (10, 1))
        self.param_update(X_new, y_new)

        # Update the IPP solution
        self.ipp_update(self.time_step)

        self.get_logger().info('Publishing waypoint {}'.format(self.time_step))
        # log all the waypoints
        for i in range(self.num_placements):
            self.get_logger().info('Waypoint {}: {}'.format(i, self.sgp_sol_sp[i]))

        # Increment the time step
        self.time_step += 1

        # Destroy the timer if all waypoints have been published
        if self.time_step >= self.num_placements:
            self.destroy_timer(self.timer)
        

def main(args=None):
    print('Starting online IPP mission')
    rclpy.init(args=args)

    xx = np.linspace(0, 10, 25)
    yy = np.linspace(0, 10, 25)
    X_train = np.array(np.meshgrid(xx, yy)).T.reshape(-1, 2)
    mission = OnlineIPP(X_train)

    rclpy.spin(mission)
    mission.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)
