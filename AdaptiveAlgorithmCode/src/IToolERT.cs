namespace AdaptiveAlgorithmCode.src
{
    /// <summary>
    /// Возвращает результат измерения за один пакет
    /// </summary>
    /// <param name="quadruples">Измеренные квадруполи</param>
    /// <param name="outputCurrent">Ток на генераторных электродах (мА)</param>
    /// <param name="outputVoltage">Напряжение на генераторных электродах (V)</param>
    /// <param name="channel">Индекс измерительного канала</param>
    /// <param name="pulseInputVoltage">Напряжение на измерительных электродах в импульсе</param>
    /// <param name="pauseInputVoltage">Напряжение на измерительных электродах в паузе между импульсами</param>
    /// <param name="coefficientOfVariation">Коэффициент вариации в "%"</param>
    /// <param name="num_package">Номер текущего пакета</param>
    /// <param name="count_package">Общее количество пакетов</param>
    /// <returns></returns>
    public delegate bool NewDataHandler( Quadruple[] quadruples, float[] outputCurrent, float[] outputVoltage,
        byte[] channel, float[] pulseInputVoltage, float[] pauseInputVoltage, float[] coefficientOfVariation, int num_package, int count_package );

    /// <summary>
    /// Интерфейс работы с прибором ERT
    /// </summary>
    public interface IToolERT
    {
        /// <summary>
        /// 
        /// </summary>
        /// <param name="quadruples"></param>
        /// <param name="new_data_event"></param>
        /// <param name="activeStatesOfElectrodes">if passed null - all electrodes are active</param>
        /// <returns></returns>
        void Measure( Quadruple[] quadruples, NewDataHandler new_data_event, bool[] activeStatesOfElectrodes );
    }
}
