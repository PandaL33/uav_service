"""MQTT 报文体积适配。

平台 broker 单包上限 1MB，超限报文会被直接断开连接（表现为 MQTT_ERR_CONN_LOST），
因此 BuildMap 报文里的 picBase64 需要按上限压缩。
"""

import base64
import io

from PIL import Image

# 实测 1020KB 可正常收发，1050KB 必被断开；留少量余量
MQTT_MAX_PAYLOAD = 1020 * 1024


def encode_map_image(image_path, overhead_bytes=0, max_payload=MQTT_MAX_PAYLOAD):
    """生成 picBase64，必要时压缩以适配 MQTT 单包上限，保持像素尺寸不变。

    Args:
        image_path: 地图图片路径。
        overhead_bytes: 报文除 picBase64 外的字节数，用于估算总包体大小。
        max_payload: 单包上限（字节）。

    Returns:
        (base64 字符串, 处理说明)。压不进上限时返回 ("", 说明)。
    """
    def _fits(data):
        return len(base64.b64encode(data)) + overhead_bytes <= max_payload

    with open(image_path, 'rb') as fp:
        raw = fp.read()
    if _fits(raw):
        return base64.b64encode(raw).decode(), '原始 PNG'

    with Image.open(image_path) as img:
        gray = img.convert('L')

        buf = io.BytesIO()
        gray.save(buf, format='PNG', optimize=True)
        if _fits(buf.getvalue()):
            return base64.b64encode(buf.getvalue()).decode(), '灰度 PNG'

        # 栅格图颜色很少，量化后无损压缩率更高；dither=0 避免抖动产生噪点
        for colors in (8, 4, 2):
            buf = io.BytesIO()
            gray.quantize(colors=colors, dither=0).save(buf, format='PNG', optimize=True)
            if _fits(buf.getvalue()):
                return base64.b64encode(buf.getvalue()).decode(), f'{colors} 色灰度 PNG'

        buf = io.BytesIO()
        gray.point(lambda v: 255 if v > 127 else 0, '1').save(buf, format='PNG', optimize=True)
        if _fits(buf.getvalue()):
            return base64.b64encode(buf.getvalue()).decode(), '二值 PNG'

    return '', '超过 MQTT 单包上限，picBase64 置空（平台可按 fileId 取图）'
