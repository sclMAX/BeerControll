package beercontroll;

public class ArduinoData {

    private int licorTemp = 0;
    private int maceradorTemp = 0;
    private int maceradorQuemador = 0;
    private int licorQuemador = 0;

    public ArduinoData(int licorTemp, int maceradorTemp, int maceradorQuemador, int licorQuemador) {
        this.licorQuemador = licorQuemador;
        this.licorTemp = licorTemp;
        this.maceradorQuemador = maceradorQuemador;
        this.maceradorTemp = maceradorTemp;
    }

    public ArduinoData() {
    }

    @Override
    public String toString() {
        return "ArduinoData{" + "licorTemp=" + licorTemp + ", maceradorTemp=" + maceradorTemp + ", maceradorQuemador=" + maceradorQuemador + ", licorQuemador=" + licorQuemador + '}';
    }

    public int getLicorTemp() {
        return licorTemp;
    }

    public void setLicorTemp(int licorTemp) {
        this.licorTemp = licorTemp;
    }

    public int getMaceradorTemp() {
        return maceradorTemp;
    }

    public void setMaceradorTemp(int maceradorTemp) {
        this.maceradorTemp = maceradorTemp;
    }

    public boolean isMaceradorQuemador() {
        return maceradorQuemador > 0;
    }

    public void setMaceradorQuemador(int maceradorQuemador) {
        this.maceradorQuemador = maceradorQuemador;
    }

    public boolean isLicorQuemador() {
        return licorQuemador > 0;
    }

    public void setLicorQuemador(int licorQuemador) {
        this.licorQuemador = licorQuemador;
    }

}
