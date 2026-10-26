def _unique_payloads(symbols):
    payloads = []
    for symbol in symbols:
        try:
            payload = symbol.data.decode('utf-8')
        except UnicodeDecodeError:
            continue
        if payload not in payloads:
            payloads.append(payload)
    return payloads


def _decode(image, decoder, qr_symbol):
    try:
        return _unique_payloads(decoder(image, symbols=[qr_symbol]))
    except Exception:
        return []


def decode_qr_payloads(frame, decoder, qr_symbol):
    payloads = _decode(frame, decoder, qr_symbol)
    if payloads:
        return payloads

    try:
        import cv2
    except Exception:
        return []

    height, width = frame.shape[:2]
    if height > 20 and width > 20:
        roi = frame[
            int(height * 0.18):int(height * 0.82),
            int(width * 0.18):int(width * 0.82),
        ]
        if roi.size:
            upscaled = cv2.resize(
                roi,
                None,
                fx=2.0,
                fy=2.0,
                interpolation=cv2.INTER_CUBIC,
            )
            payloads = _decode(upscaled, decoder, qr_symbol)
            if payloads:
                return payloads

    threshold = cv2.adaptiveThreshold(
        frame,
        255,
        cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
        cv2.THRESH_BINARY,
        31,
        5,
    )
    payloads = _decode(threshold, decoder, qr_symbol)
    if payloads:
        return payloads

    return []


class QrSubmissionGate:
    def __init__(self):
        self.active = False
        self.submitted_payloads = set()

    def set_game_active(self, active):
        if active and not self.active:
            self.submitted_payloads.clear()
        self.active = active

    def accept(self, payload):
        normalized = payload.strip()
        if (
            not self.active
            or not normalized
            or normalized in self.submitted_payloads
        ):
            return None
        self.submitted_payloads.add(normalized)
        return normalized
