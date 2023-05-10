namespace AdaptiveAlgorithmCode.src
{
    public class QuadMeasurement
    {
        public float OutputVoltage;
        public float OutputCurrent;
        public float PulseInputVoltage;
        public float PauseInputVoltage;
        public float CoefficientOfVariation;
        public float Resistance;

        public QuadMeasurement( float outputVoltage, float outputCurrent, float pulseInputVoltage,
            float pauseInputVoltage,
            float coefficientOfVariation, float resistance )
        {
            OutputVoltage = outputVoltage;
            PulseInputVoltage = pulseInputVoltage;
            PauseInputVoltage = pauseInputVoltage;
            CoefficientOfVariation = coefficientOfVariation;
            Resistance = resistance;
            OutputCurrent = outputCurrent;
        }
    }
}
