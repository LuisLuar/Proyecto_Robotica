import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def plot_correct_coordinate_systems():
    """Visualización con sistema de cámara correcto (Z hacia adelante)"""
    fig = plt.figure(figsize=(14, 10))
    ax = fig.add_subplot(111, projection='3d')

    # Configuración de ejes
    ax.set_xlim([-1, 2])
    ax.set_ylim([-1, 2])
    ax.set_zlim([0, 3])
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.set_title('Sistemas de Referencia con Orientación Correcta de Cámara', fontsize=14, pad=20)

    # 1. Sistema de la Cámara (CORREGIDO)
    T_cam = np.eye(4)
    T_cam[:3, 3] = [0, 0, 2.5]  # Posición de la cámara
    
    # Orientación correcta:
    # - X: derecha
    # - Y: abajo 
    # - Z: hacia adelante (plano de la imagen)
    T_cam[:3, :3] = np.array([[1, 0, 0],
                              [0, -1, 0],  # Invertido Y porque en imágenes Y crece hacia abajo
                              [0, 0, -1]]) # Z apunta hacia el plano XY (hacia adelante)
    
    draw_axes(ax, T_cam, ['red', 'green', 'blue'], 'Cámara')

    # 2. Tag Origen (suelo)
    T_tag1 = np.eye(4)
    T_tag1[:3, 3] = [0.5, 0.5, 0]  # Posición en el suelo
    draw_axes(ax, T_tag1, ['magenta', 'yellow', 'cyan'], 'Tag 1 (Origen)')

    # 3. Tag Secundario (suelo)
    T_tag2 = np.eye(4)
    T_tag2[:3, 3] = [1.5, 1.0, 0]
    T_tag2[:3, :3] = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]])  # Rotado 90° en Z
    draw_axes(ax, T_tag2, ['orange', 'lime', 'purple'], 'Tag 2')

    # Conexiones entre sistemas
    def draw_connection(ax, T1, T2, color, label):
        ax.plot([T1[0,3], T2[0,3]], 
                [T1[1,3], T2[1,3]], 
                [T1[2,3], T2[2,3]], 
                color=color, linestyle='--', linewidth=1)
        mid_point = (T1[:3,3] + T2[:3,3])/2
        ax.text(mid_point[0], mid_point[1], mid_point[2], label, fontsize=9)

    draw_connection(ax, T_cam, T_tag1, 'black', 'Cámara → Tag1')
    draw_connection(ax, T_cam, T_tag2, 'black', 'Cámara → Tag2')
    draw_connection(ax, T_tag1, T_tag2, 'red', 'Tag1 → Tag2')

    plt.tight_layout()
    plt.show()

def draw_axes(ax, T, colors, label):
    """Dibuja los ejes de un sistema de coordenadas."""
    axis_length = 0.3
    ax.quiver(T[0,3], T[1,3], T[2,3], 
              T[0,0], T[1,0], T[2,0], color=colors[0], length=axis_length)
    ax.quiver(T[0,3], T[1,3], T[2,3], 
              T[0,1], T[1,1], T[2,1], color=colors[1], length=axis_length)
    ax.quiver(T[0,3], T[1,3], T[2,3], 
              T[0,2], T[1,2], T[2,2], color=colors[2], length=axis_length)
    ax.text(T[0,3], T[1,3], T[2,3], label, fontsize=10, zorder=10)

if __name__ == '__main__':
    plot_correct_coordinate_systems()