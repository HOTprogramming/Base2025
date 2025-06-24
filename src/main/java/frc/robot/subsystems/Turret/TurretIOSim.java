package frc.robot.subsystems.Turret;
import edu.wpi.first.math.*;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.NumericalIntegration;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.Constants;


public class TurretIOSim implements TurretIO {
  private static final double moi = 1.2;
  private static final DCMotor gearbox =
      DCMotor.getKrakenX60Foc(1).withReduction(TurretIOKraken.reduction);
  private static final Matrix<N2, N2> A =
      MatBuilder.fill(
          Nat.N2(),
          Nat.N2(),
          0,
          1,
          0,
          -gearbox.KtNMPerAmp / (gearbox.KvRadPerSecPerVolt * gearbox.rOhms * moi));
  private static final Vector<N2> B = VecBuilder.fill(0, gearbox.KtNMPerAmp / moi);

  // Turret sim
  private Vector<N2> simState;
  private double inputTorqueCurrent = 0.0;
  private double appliedVolts = 0.0;

  public TurretIOSim() {
    simState = VecBuilder.fill(0.0, 0.0);
  }

  @Override
  public void updateStatusSignals(TurretIOInputs inputs) {
    update(Constants.loopPeriodSecs);

    inputs.data =
        new TurretIOData(
            true,
            simState.get(0),
            simState.get(1),
            appliedVolts,
            Math.copySign(inputTorqueCurrent, appliedVolts),
            Math.copySign(inputTorqueCurrent, appliedVolts),
            0.0);
  }

  @Override
  public void runTorqueCurrent(double current) {
    inputTorqueCurrent = current;
    appliedVolts = gearbox.getVoltage(gearbox.getTorque(inputTorqueCurrent), simState.get(1, 0));
    appliedVolts = MathUtil.clamp(appliedVolts, -12.0, 12.0);
  }

  @Override
  public void stop() {
    System.out.println("TurretIOSim...Stopped");
    runTorqueCurrent(0.0);
  }

  private void update(double dt) {
    inputTorqueCurrent =
        MathUtil.clamp(inputTorqueCurrent, -gearbox.stallCurrentAmps, gearbox.stallCurrentAmps);
    Matrix<N2, N1> updatedState =
        NumericalIntegration.rkdp(
            (Matrix<N2, N1> x, Matrix<N1, N1> u) -> A.times(x).plus(B.times(u)),
            simState,
            VecBuilder.fill(inputTorqueCurrent * 15), // Magic constant of doom
            dt);
    simState = VecBuilder.fill(updatedState.get(0, 0), updatedState.get(1, 0));
  }
}