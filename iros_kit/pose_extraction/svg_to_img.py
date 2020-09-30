import tempfile
import cairosvg
import numpy as np
import cv2


def svg_to_numpy(svg_file):
    with tempfile.TemporaryFile() as f:
        cairosvg.svg2png(
            bytestring=open(svg_file).read(),
            write_to=f
        )
        f.seek(0)
        return cv2.imdecode(np.fromfile(f, np.uint8), cv2.IMREAD_GRAYSCALE)


def main():
    img = svg_to_numpy('IROS2020_KitLayout practice.svg')
    cv2.imshow('', img)
    cv2.waitKey()


if __name__ == '__main__':
    main()
