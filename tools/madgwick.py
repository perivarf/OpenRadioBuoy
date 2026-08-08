"""Madgwick 6-akse AHRS (gyro + accel, ingen magnetometer).

Identisk regnestykke med analysis.cpp om bord - se kommentaren i update(). Det er
et krav, ikke en tilfeldighet: hele poenget med offline-analysen er å kunne
sammenligne mot det bøya selv regnet ut, og da må filteret være bit-for-bit likt.

Grensesnittet er delt med kalman.py:

    f = Madgwick(beta=0.2)
    f.init_from_accel(ax, ay, az)          # ett sample -> startorientering
    f.update(gx, gy, gz, ax, ay, az, dt)   # gyro i rad/s, accel i vilkårlig enhet
    f.q                                    # [w, x, y, z], body -> verden

Accel-enheten er likegyldig (vektoren normaliseres), gyro MÅ være rad/s og dt
sekunder.
"""

import math

from rotation import quat_from_accel


class Madgwick:
    """Madgwick 6-akse AHRS. Tilstanden er kun quaternionen."""

    def __init__(self, beta=0.2):
        # beta = hvor hardt accel-retningen trekker quaternionen mot loddrett.
        # Høy verdi følger accel tett (og arver dens støy), lav verdi lener seg
        # på gyroen (og arver dens drift).
        self.beta = float(beta)
        self._q = [1.0, 0.0, 0.0, 0.0]

    @property
    def q(self):
        """Orienteringen som [w, x, y, z], body -> verden."""
        return self._q

    def init_from_accel(self, ax, ay, az):
        """Sett startorienteringen fra ett accel-sample (yaw = 0).

        Uten dette ville filteret startet på identitet og brukt de første
        sekundene på å svinge seg inn - en transient som ÷ω⁴ deretter blåser opp
        til en falsk lavfrekvenstopp i elevasjonsspekteret."""
        self._q = quat_from_accel(ax, ay, az)

    def update(self, gx, gy, gz, ax, ay, az, dt):
        """Ett steg. Muterer den interne quaternionen.

        Utskrevet i sin helhet (ingen numpy, ingen mellomvariabler samlet i
        vektorer) fordi den skal kunne leses side om side med analysis.cpp linje
        for linje. Gradienten s0..s3 er den analytiske deriverte av
        gravitasjonsfeilen - Madgwicks poeng er nettopp at den finnes i lukket
        form, så ett gradientsteg per sample er nok."""
        q0, q1, q2, q3 = self._q
        qdot1 = 0.5 * (-q1 * gx - q2 * gy - q3 * gz)
        qdot2 = 0.5 * (q0 * gx + q2 * gz - q3 * gy)
        qdot3 = 0.5 * (q0 * gy - q1 * gz + q3 * gx)
        qdot4 = 0.5 * (q0 * gz + q1 * gy - q2 * gx)

        if not (ax == 0.0 and ay == 0.0 and az == 0.0):
            recip = 1.0 / math.sqrt(ax * ax + ay * ay + az * az)
            ax *= recip; ay *= recip; az *= recip
            _2q0, _2q1, _2q2, _2q3 = 2.0 * q0, 2.0 * q1, 2.0 * q2, 2.0 * q3
            _4q0, _4q1, _4q2 = 4.0 * q0, 4.0 * q1, 4.0 * q2
            _8q1, _8q2 = 8.0 * q1, 8.0 * q2
            q0q0, q1q1, q2q2, q3q3 = q0 * q0, q1 * q1, q2 * q2, q3 * q3
            s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay
            s1 = (_4q1 * q3q3 - _2q3 * ax + 4.0 * q0q0 * q1 - _2q0 * ay
                  - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az)
            s2 = (4.0 * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay
                  - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az)
            s3 = 4.0 * q1q1 * q3 - _2q1 * ax + 4.0 * q2q2 * q3 - _2q2 * ay
            recip = 1.0 / math.sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3)
            s0 *= recip; s1 *= recip; s2 *= recip; s3 *= recip
            qdot1 -= self.beta * s0
            qdot2 -= self.beta * s1
            qdot3 -= self.beta * s2
            qdot4 -= self.beta * s3

        q0 += qdot1 * dt; q1 += qdot2 * dt; q2 += qdot3 * dt; q3 += qdot4 * dt
        recip = 1.0 / math.sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3)
        self._q = [q0 * recip, q1 * recip, q2 * recip, q3 * recip]
        return self._q
