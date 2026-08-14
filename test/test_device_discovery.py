from pathlib import Path

from astra_rgb_camera.rgb_camera_node import find_video_devices


def _make_video_device(
    root: Path, name: str, vendor_id: str, product_id: str
) -> tuple[Path, Path]:
    usb_device = root / "sys" / "devices" / "usb1" / "1-2"
    video_device = usb_device / "1-2:1.0" / "video4linux" / name
    video_device.mkdir(parents=True)
    (usb_device / "idVendor").write_text(vendor_id, encoding="ascii")
    (usb_device / "idProduct").write_text(product_id, encoding="ascii")

    class_root = root / "sys" / "class" / "video4linux"
    class_root.mkdir(parents=True, exist_ok=True)
    (class_root / name).symlink_to(video_device, target_is_directory=True)

    dev_root = root / "dev"
    dev_root.mkdir(exist_ok=True)
    (dev_root / name).touch()
    return class_root, dev_root


def test_find_astra_rgb_device(tmp_path):
    class_root, dev_root = _make_video_device(tmp_path, "video3", "2bc5", "050f")

    assert find_video_devices(
        sys_class_root=class_root, dev_root=dev_root
    ) == [str(dev_root / "video3")]


def test_ignore_other_usb_product(tmp_path):
    class_root, dev_root = _make_video_device(tmp_path, "video0", "2bc5", "060f")

    assert find_video_devices(sys_class_root=class_root, dev_root=dev_root) == []
