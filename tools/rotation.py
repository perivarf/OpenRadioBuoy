"""Generell rotasjonsmatematikk for ORB-analysen: quaternioner, Euler-vinkler og
rotasjonsmatriser.

Rent geometri - INGEN domenekonstanter (ingen g, ingen sensor-enheter, ingen
settings.h-speiling). Det er skillet som gjør at fila kan gjenbrukes av både
madgwick.py, kalman.py og postprocess.py uten sirkulære importer: filtrene trenger
quaternion-algebra, analysen trenger projeksjoner, og ingen av dem trenger å vite
om den andre.

Konvensjoner, like over hele prosjektet og identiske med analysis.cpp:
  - quaternion er [w, x, y, z] (Hamilton, skalar først)
  - q roterer BODY -> VERDEN, altså v_verden = R(q) · v_body
  - Euler er roll om x, pitch om y, yaw = 0 (6-akse IMU kan ikke observere yaw)

Vektoriserte og skalare varianter finnes side om side: quat_matrix() tar én
quaternion og gir 3x3, quat_matrix_array() tar N stk og gir N x 3 x 3. De to gir
nøyaktig samme tall - den vektoriserte finnes fordi den er størrelsesordener
raskere når hele økta skal roteres på én gang.
"""

import math

import numpy as np


def quat_from_accel(ax, ay, az):
    """Quaternion fra ETT accel-sample, yaw = 0 (identisk med analysis.cpp).

    Enhetsuavhengig: kun forholdene mellom aksene brukes, så mg og m/s² gir
    samme svar. Brukes til å initialisere filtrene fra første måling i stedet for
    å starte på identitet - da slipper man en konvergenstransient i starten."""
    roll = math.atan2(ay, az)
    pitch = math.atan2(-ax, math.sqrt(ay * ay + az * az))
    return quat_from_roll_pitch(roll, pitch)


def quat_from_roll_pitch(roll, pitch):
    """Quaternion [w,x,y,z] fra roll/pitch med yaw = 0."""
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    return [cp * cr, cp * sr, sp * cr, -sp * sr]


def roll_pitch_from_quat(qw, qx, qy, qz):
    """(roll, pitch) i radianer fra quaternion. Motsatt vei av
    quat_from_roll_pitch, og eksakt invers av den når yaw = 0."""
    sinr = 2.0 * (qw * qx + qy * qz)
    cosr = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr, cosr)
    sinp = 2.0 * (qw * qy - qz * qx)
    sinp = max(-1.0, min(1.0, sinp))          # klem mot avrundingsfeil i asin
    return roll, math.asin(sinp)


def quat_mul(a, b):
    """Hamilton-produkt for [w,x,y,z]."""
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.array([aw * bw - ax * bx - ay * by - az * bz,
                     aw * bx + ax * bw + ay * bz - az * by,
                     aw * by - ax * bz + ay * bw + az * bx,
                     aw * bz + ax * by - ay * bx + az * bw])


def quat_exp(v):
    """Rotasjonsvektor (rad) -> quaternion. Liten-vinkel-grenen unngår 0/0."""
    th = math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])
    if th < 1e-12:
        return np.array([1.0, 0.5 * v[0], 0.5 * v[1], 0.5 * v[2]])
    s = math.sin(0.5 * th) / th
    return np.array([math.cos(0.5 * th), v[0] * s, v[1] * s, v[2] * s])


def quat_normalize(q):
    """Normaliser [w,x,y,z] til enhetslengde. Returnerer liste."""
    n = math.sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3])
    if n <= 0.0:
        return [1.0, 0.0, 0.0, 0.0]
    return [q[0] / n, q[1] / n, q[2] / n, q[3] / n]


def quat_matrix(q):
    """R(q) body -> verden, 3x3, for ÉN quaternion."""
    qw, qx, qy, qz = q
    return np.array([
        [1.0 - 2.0 * (qy * qy + qz * qz), 2.0 * (qx * qy - qw * qz), 2.0 * (qx * qz + qw * qy)],
        [2.0 * (qx * qy + qw * qz), 1.0 - 2.0 * (qx * qx + qz * qz), 2.0 * (qy * qz - qw * qx)],
        [2.0 * (qx * qz - qw * qy), 2.0 * (qy * qz + qw * qx), 1.0 - 2.0 * (qx * qx + qy * qy)]])


def quat_matrix_array(q):
    """R(q) body -> verden for N quaternioner: (N,4) -> (N,3,3).
    Samme tall som quat_matrix(), men vektorisert over hele økta."""
    q = np.asarray(q, dtype=np.float64)
    qw, qx, qy, qz = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    R = np.empty((len(q), 3, 3), dtype=np.float64)
    R[:, 0, 0] = 1.0 - 2.0 * (qy * qy + qz * qz)
    R[:, 0, 1] = 2.0 * (qx * qy - qw * qz)
    R[:, 0, 2] = 2.0 * (qx * qz + qw * qy)
    R[:, 1, 0] = 2.0 * (qx * qy + qw * qz)
    R[:, 1, 1] = 1.0 - 2.0 * (qx * qx + qz * qz)
    R[:, 1, 2] = 2.0 * (qy * qz - qw * qx)
    R[:, 2, 0] = 2.0 * (qx * qz - qw * qy)
    R[:, 2, 1] = 2.0 * (qy * qz + qw * qx)
    R[:, 2, 2] = 1.0 - 2.0 * (qx * qx + qy * qy)
    return R


def world_z(q, ax, ay, az):
    """Z-komponenten av vektoren rotert til verdensramme: (R(q)·a)_z.

    Skrevet ut som tre ledd i stedet for full matrise-multiplikasjon fordi bare
    én komponent trengs - dette kalles én gang per rå IMU-rad. Tyngdekraften
    trekkes IKKE fra her; det er et domenevalg som hører hjemme hos kalleren."""
    qw, qx, qy, qz = q
    return (2.0 * (qx * qz - qw * qy) * ax
            + 2.0 * (qy * qz + qw * qx) * ay
            + (1.0 - 2.0 * (qx * qx + qy * qy)) * az)


def skew(v):
    """Skjevsymmetrisk matrise slik at skew(a) @ b == np.cross(a, b)."""
    return np.array([[0.0, -v[2], v[1]],
                     [v[2], 0.0, -v[0]],
                     [-v[1], v[0], 0.0]])
