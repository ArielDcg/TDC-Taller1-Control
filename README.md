# Taller 1 - Teoría de Control (TDC)
## Control PID y Control Robusto $H_\infty$

Este repositorio contiene todo el código MATLAB, simulaciones y modelos de Simulink desarrollados para el **Taller 1 de Teoría de Control**, abarcando tanto el diseño clásico (PID) como el diseño moderno/robusto ($H_\infty$).

---

### 📂 Estructura del Proyecto

```
Taller1/
├── main.mlx                       # Live Script interactivo con desarrollo y pruebas
├── modelo_lin.mat                 # Modelo linealizado del sistema en espacio de estados
├── T1.m                           # Script de análisis preliminar y lugar de raíces
└── Final/
    └── Final/                     # Implementación final de controladores y simulaciones
        ├── PIDComplelto.m         # Sintonización completa del control PID
        ├── PID_lat.m              # Controlador PID lateral
        ├── PID_SIMULINK.slx       # Modelo Simulink de lazo cerrado PID
        ├── T1_Hinf_Dir.m          # Control H-infinito direccional (yaw)
        ├── T1_Hinf_Dir_v2.m       # Control H-infinito direccional v2 (con filtro washout)
        ├── T1_Hinf_Lat.m          # Control H-infinito lateral (roll)
        ├── T1_Hinf_Long.m         # Control H-infinito longitudinal (pitch)
        ├── SimuHInf.slx           # Simulación Simulink para H-infinito
        ├── H_inf_SIMULINK.slx     # Modelo de simulación general H-infinito
        ├── generate_hinf_plots.m  # Generación automatizada de curvas sigma y escalón
        ├── get_pid_metrics.m      # Cálculo de métricas temporales PID
        ├── get_all_metrics.m      # Comparativa de métricas entre PID y H-infinito
        └── modelo_lin.mat         # Datos del modelo linealizado
```

---

### 🚀 Flujo de Trabajo y Colaboración

Para mantener sincronizado el trabajo del equipo:

1. **Antes de empezar a programar:**
   ```bash
   git pull origin main
   ```

2. **Al realizar modificaciones:**
   ```bash
   git add .
   git commit -m "Descripción clara de los cambios realizados"
   ```

3. **Subir los cambios al repositorio:**
   ```bash
   git push origin main
   ```

> ⚠️ **Archivos de Simulink (`.slx` y `.mat`):**
> Al ser binarios, se recomienda coordinar con el equipo antes de modificar los diagramas para evitar conflictos de mezcla (*merge conflicts*).
