import numpy as np
import cv2
import onnxruntime as ort

session = ort.InferenceSession("yolov4.onnx")
input_size = 416

# Load class names
with open("coco.names", "r") as f:
    class_names = [c.strip() for c in f.readlines()]


def sigmoid(x):
    return 1 / (1 + np.exp(-x))


def image_preprocess(image, target_size=[input_size, input_size], gt_boxes=None):

    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

    ih, iw = target_size
    h, w, _ = image.shape

    scale = min(iw / w, ih / h)
    nw, nh = int(scale * w), int(scale * h)
    image_resized = cv2.resize(image, (nw, nh))

    image_padded = np.full(shape=[ih, iw, 3], fill_value=128.0)
    dw, dh = (iw - nw) // 2, (ih - nh) // 2
    image_padded[dh : nh + dh, dw : nw + dw, :] = image_resized
    image_padded = image_padded / 255.0

    if gt_boxes is None:
        return image_padded

    else:
        gt_boxes[:, [0, 2]] = gt_boxes[:, [0, 2]] * scale + dw
        gt_boxes[:, [1, 3]] = gt_boxes[:, [1, 3]] * scale + dh
        return image_padded, gt_boxes


def make_inference(data):
    input_name = session.get_inputs()[0].name
    output_name = session.get_outputs()[0].name
    return session.run([output_name], {input_name: data})[0]


def postprocess(output):

    boxes, confidences, class_ids = [], [], []
    anchor_w = [12, 19, 40]
    anchor_h = [16, 36, 28]

    row_no = 0
    col_no = 0
    anchor_no = 0

    for row in output[0]:  # batch size = 1
        col_no = 0
        for col in row:
            anchor_no = 0
            for anchor in col:
                scores = anchor[5:]
                class_id = np.argmax(scores)
                confidence = scores[class_id] * anchor[4]  # obj_conf * class_conf
                if confidence > 0.2:
                    cx, cy, w, h = anchor[0:4]
                    w = int(anchor_w[anchor_no] * np.exp(w))
                    h = int(anchor_h[anchor_no] * np.exp(h))
                    x = int((sigmoid(cx) + col_no) / 52 * input_size - w / 2)
                    y = int((sigmoid(cy) + row_no) / 52 * input_size - h / 2)

                    boxes.append([x, y, w, h])
                    confidences.append(float(confidence))
                    class_ids.append(class_id)

                    print(f"DETECT {class_names[class_id]}!")

                anchor_no = anchor_no + 1

            col_no = col_no + 1
        row_no = row_no + 1

    results = []
    for i in range(0, len(boxes)):
        results.append((boxes[i], class_ids[i], confidences[i]))
    return results


def draw_bbox(image, results):
    for box, class_id, conf in results:
        x, y, w, h = box
        cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)
        label = f"{class_names[class_id]}: {conf:.2f}"
        cv2.putText(
            image, label, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2
        )

    return image


original_image = cv2.imread("input.jpg")

image_pp = image_preprocess(np.copy(original_image))
image_data = image_pp[np.newaxis, ...].astype(np.float32)

inference = make_inference(image_data)
bbox_info = postprocess(inference)
final_image = draw_bbox(image_pp, bbox_info)

cv2.imshow("Result", final_image)
cv2.waitKey(0)
cv2.destroyAllWindows()
