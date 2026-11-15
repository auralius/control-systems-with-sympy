#pragma once

class ButterworthHPF2 {
    float q = 0.0f, q2 = 0.0f;
    float x1 = 0.0f, x2 = 0.0f;
    float y1 = 0.0f, y2 = 0.0f;

public:
    void configure(float Ts, float wc) {
        q = Ts * wc;
        q2 = q * q;
        reset();
    }

    void reset() {
        x1 = x2 = 0.0f;
        y1 = y2 = 0.0f;
    }

    float step(float x0) {
        const float y0 = (2*1.4142135623730951f*q*y2 - 2*q2*y1 - q2*y2 + 4*x0 - 8*x1 + 4*x2 + 8*y1 - 4*y2)/(2*1.4142135623730951f*q + q2 + 4);
        x2 = x1; x1 = x0;
        y2 = y1; y1 = y0;
        return y0;
    }
};
