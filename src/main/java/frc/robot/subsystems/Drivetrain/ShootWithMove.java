package frc.robot.subsystems.Drivetrain;
///This class contains a method for finding the angle to shoot at a target while the robot is moving. The input values are: the x and y components of the 
/// robot velocity (Vrx and Vry), the position of the robot (xr and yr), the position of the target (xt and yt), and the projectile firing speed (Vp).
/// The output value is in radians between 0 and 2pi. It will ouput -1 for impossible shots. GRAVITY HAS NOT YET BEEN IMPLIMENTED, PLEASE BE PATIENT!!!

public class ShootWithMove {
    public static double ShootAngleHCalc(double Vrx, double Vry, double xr, double yr, double xt, double yt, double Vp){ //May return any value between 0 and 2pi. Any value OUTSIDE of this range indicates an error, and that the shot cannot be made.
        double Vr = Math.sqrt(Vrx * Vrx + Vry * Vry); //Find total robot velocity from x and y compontents of velocity
        double Thetar = Math.atan(Vry / Vrx); //Find angle that the robot is moving at from x and y compontents of velocity
        if (Vrx < 0) { //atan doesn't cover all angles, but this takes care of that
            Thetar += Math.PI;
        }

        double D = Math.sqrt((xt - xr) * (xt - xr) + (yt - yr) * (yt - yr)); //Find distance to target using robot and target positions
        double Thetat = Math.atan((yt - yr) / (xt - xr)); //Find angle to target using robot and target positions
        if (xt - xr < 0) { //atan doesn't cover all angles, but this takes care of that 
            Thetat += Math.PI;
        }

        double Tdiff = Math.cos(Thetar - Thetat); //A value used a lot, precalculated to save time and my fingers
        double root = Vr * Vr * (Tdiff * Tdiff - 1) + Vp * Vp; //See above comment
        if (root < 0) { //Testing for imaginary time, outputting -1 as an error value if that happens
            return(-1); //Error value for impossible to hit shot
        } else if (Vr * Tdiff + Math.sqrt(root) < 0){ //Testing for negative time, outputting -1 as an error value if that happens
            return(-1); //Error value for impossible to hit shot
        }

        double t = D / (Vr * Tdiff + Math.sqrt(root)); //Calculating time to hit target, used in calculating angle of shot
        double Thetap = Math.atan(((yt - yr) / t - Vry) / ((xt - xr) / t - Vrx)); //Calculating the angle to shoot at

        if ((xt - xr) / t - Vrx < 0) { //atan doesn't cover all angles, but this takes care of that
            Thetap += Math.PI;
        }
        if (Thetap < 0) { //keeps angle value in range of 0 to 2 pi radians for ease of use (YOU'RE WELCOME)
            Thetap += 2 * Math.PI;
        }
        return(Thetap);
        //test
    }
}