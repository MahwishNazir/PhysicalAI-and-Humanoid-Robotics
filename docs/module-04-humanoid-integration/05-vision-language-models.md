---
title: "Vision-Language Models"
sidebar_label: "Vision-Language Models"
sidebar_position: 5
description: "Integrating visual perception with language understanding for robotics"
tags: [vision-language, clip, vqa, visual-grounding, multimodal]
keywords: [vision-language models, CLIP, BLIP, visual grounding, VQA, multimodal AI]
difficulty: intermediate
estimated_time: "2.5 hours"
prerequisites: ["Chapter 4: Natural Language to Robot Actions"]
---

# Vision-Language Models

*Content coming soon. This chapter will cover:*

## Introduction to Vision-Language Models

### What are VLMs?

Models that jointly understand images and text:

```
Input: Image + Text
Output: Aligned representation / Answer / Detection
```

**Key Capabilities:**
- **Zero-shot image classification**: "Is this a cup?" without training
- **Visual Question Answering**: "How many boxes are on the table?"
- **Image-text matching**: Find images matching "a red car"
- **Visual grounding**: Locate "the laptop on the right"

### Why VLMs for Robotics?

Traditional approach:
```
Train detector for cups → Train detector for bottles → Train detector for plates → ...
```

VLM approach:
```
Detect "any object the user describes" without training
```

**Advantages:**
- **Open-vocabulary**: Detect novel objects
- **Language-driven**: "Find the tall green bottle"
- **Flexible**: Adapt to new tasks without retraining
- **Context-aware**: "The cup the user is pointing at"

## CLIP: Contrastive Language-Image Pre-training

### Architecture

```
Image Encoder (Vision Transformer) → Image Embedding
                                            ↓
                                      Similarity Score
                                            ↓
Text Encoder (Transformer) → Text Embedding
```

Trained on 400M image-text pairs from the internet.

### How CLIP Works

**Training:**
- Learns to maximize similarity between matching image-text pairs
- Minimize similarity for non-matching pairs

**Inference (Zero-Shot Classification):**

```python
import clip
import torch
from PIL import Image

# Load model
model, preprocess = clip.load("ViT-B/32", device="cuda")

# Load image
image = preprocess(Image.open("robot_view.jpg")).unsqueeze(0).to("cuda")

# Define text queries
text_queries = ["a cup", "a bottle", "a phone", "nothing"]
text = clip.tokenize(text_queries).to("cuda")

# Compute similarity
with torch.no_grad():
    image_features = model.encode_image(image)
    text_features = model.encode_text(text)

    similarity = (image_features @ text_features.T).softmax(dim=-1)

print(f"Predictions: {text_queries}")
print(f"Probabilities: {similarity[0].cpu().numpy()}")
```

### Using CLIP in ROS 2

```python
class CLIPPerceptionNode(Node):
    def __init__(self):
        super().__init__('clip_perception')
        self.bridge = CvBridge()
        self.model, self.preprocess = clip.load("ViT-B/32")

        self.image_sub = self.create_subscription(
            Image, '/camera/image', self.image_callback, 10
        )
        self.query_sub = self.create_subscription(
            String, '/object_query', self.query_callback, 10
        )

    def query_callback(self, msg):
        # User asks: "Is there a red box?"
        self.current_query = msg.data

    def image_callback(self, msg):
        # Convert ROS image to PIL
        cv_image = self.bridge.imgmsg_to_cv2(msg, "rgb8")
        pil_image = Image.fromarray(cv_image)

        # Classify with CLIP
        result = self.classify_image(pil_image, [self.current_query, "nothing"])

        # Publish result
        self.result_pub.publish(result)
```

## Visual Question Answering (VQA)

### What is VQA?

Given an image and a question, generate an answer:

```
Image: [Robot's camera view of kitchen]
Question: "How many cups are on the table?"
Answer: "3"
```

### BLIP-2: Bootstrap Language-Image Pre-training

**Architecture:**
- Vision encoder (pretrained)
- Language model (pretrained LLM)
- Q-Former: Bridges vision and language

**Usage:**

```python
from transformers import Blip2Processor, Blip2ForConditionalGeneration
from PIL import Image

processor = Blip2Processor.from_pretrained("Salesforce/blip2-opt-2.7b")
model = Blip2ForConditionalGeneration.from_pretrained("Salesforce/blip2-opt-2.7b")

image = Image.open("robot_view.jpg")
question = "What objects are on the table?"

inputs = processor(image, question, return_tensors="pt")
outputs = model.generate(**inputs)
answer = processor.decode(outputs[0], skip_special_tokens=True)

print(f"Q: {question}")
print(f"A: {answer}")
```

### VQA for Robotic Tasks

**Scene Understanding:**
```
"Are there any obstacles in front?"
"Is the door open or closed?"
"What color is the box?"
```

**Spatial Reasoning:**
```
"Which object is closest to the robot?"
"Is the cup on the left or right?"
"How many objects are there?"
```

**Task Verification:**
```
"Did I successfully place the object?"
"Is the room clean?"
"Are all objects organized?"
```

## Grounding DINO: Open-Vocabulary Object Detection

### What is Grounding DINO?

Detects objects based on text descriptions:

```
Input: Image + "red box"
Output: Bounding boxes around all red boxes
```

**Advantages over traditional detectors:**
- No training needed for new objects
- Understands complex descriptions
- Works with any object category

### Using Grounding DINO

```python
from groundingdino.util.inference import load_model, predict

# Load model
model = load_model("GroundingDINO/groundingdino/config/GroundingDINO_SwinT_OGC.py",
                   "weights/groundingdino_swint_ogc.pth")

# Load image
image = load_image("robot_view.jpg")

# Detect objects
TEXT_PROMPT = "red box . blue bottle . laptop"
boxes, logits, phrases = predict(
    model=model,
    image=image,
    caption=TEXT_PROMPT,
    box_threshold=0.35,
    text_threshold=0.25
)

# boxes contains [x1, y1, x2, y2] for each detection
# phrases contains which text prompt matched
```

### Integration with Grasp Planning

```python
def find_and_grasp(object_description):
    # 1. Capture image
    image = camera.capture()

    # 2. Detect object with Grounding DINO
    boxes, logits, phrases = detect_objects(image, object_description)

    if len(boxes) == 0:
        return False, "Object not found"

    # 3. Convert 2D box to 3D pose using depth
    depth = depth_camera.capture()
    object_pose_3d = estimate_3d_pose(boxes[0], depth)

    # 4. Plan grasp
    grasp_plan = moveit.plan_grasp(object_pose_3d)

    # 5. Execute
    success = execute_plan(grasp_plan)

    return success, "Grasped successfully"
```

## OWL-ViT: Open-World Localization

### What is OWL-ViT?

Open-vocabulary detection based on CLIP:

```python
from transformers import OwlViTProcessor, OwlViTForObjectDetection
import torch

processor = OwlViTProcessor.from_pretrained("google/owlvit-base-patch32")
model = OwlViTForObjectDetection.from_pretrained("google/owlvit-base-patch32")

image = Image.open("scene.jpg")
texts = [["a red cup", "a laptop", "a phone"]]

inputs = processor(text=texts, images=image, return_tensors="pt")
outputs = model(**inputs)

# Post-process
target_sizes = torch.Tensor([image.size[::-1]])
results = processor.post_process_object_detection(
    outputs=outputs, target_sizes=target_sizes, threshold=0.1
)

for box, score, label in zip(results[0]["boxes"], results[0]["scores"], results[0]["labels"]):
    box = [round(i, 2) for i in box.tolist()]
    print(f"Detected {texts[0][label]} with confidence {round(score.item(), 3)} at {box}")
```

## Scene Understanding

### Captioning

Generate natural language descriptions of scenes:

```python
from transformers import BlipProcessor, BlipForConditionalGeneration

processor = BlipProcessor.from_pretrained("Salesforce/blip-image-captioning-base")
model = BlipForConditionalGeneration.from_pretrained("Salesforce/blip-image-captioning-base")

image = Image.open("robot_view.jpg")
inputs = processor(image, return_tensors="pt")
outputs = model.generate(**inputs)
caption = processor.decode(outputs[0], skip_special_tokens=True)

print(f"Scene description: {caption}")
# "a kitchen table with cups and plates on it"
```

**Use cases:**
- Logging robot observations
- Providing context to LLMs
- Verifying task completion

### Semantic Segmentation with Language

```python
# CLIPSeg: Segment anything you can describe
from transformers import CLIPSegProcessor, CLIPSegForImageSegmentation

processor = CLIPSegProcessor.from_pretrained("CIDAS/clipseg-rd64-refined")
model = CLIPSegForImageSegmentation.from_pretrained("CIDAS/clipseg-rd64-refined")

image = Image.open("robot_view.jpg")
texts = ["floor", "table", "obstacles"]

inputs = processor(text=texts, images=[image]*len(texts), return_tensors="pt")
outputs = model(**inputs)

# outputs.logits: segmentation masks for each text query
```

## Spatial Reasoning

### Understanding Relationships

```python
def spatial_query(image, question):
    """
    Examples:
    - "Is the cup to the left of the bottle?"
    - "What is between the laptop and the phone?"
    - "Which object is closest to the camera?"
    """

    # Use VQA model for spatial reasoning
    answer = vqa_model(image, question)
    return answer
```

### Relative Grounding

```python
def find_relative_object(image, description):
    """
    Description examples:
    - "the cup on the left"
    - "the tallest bottle"
    - "the box next to the laptop"
    """

    # Detect all objects
    all_detections = detect_all_objects(image)

    # Use LLM + detections to resolve relative reference
    prompt = f"""
    Detected objects: {all_detections}
    Find: "{description}"
    Return the object ID that best matches.
    """

    selected_id = llm.query(prompt)
    return all_detections[selected_id]
```

## Multimodal Integration with LLMs

### GPT-4 Vision

```python
import openai

response = openai.ChatCompletion.create(
    model="gpt-4-vision-preview",
    messages=[
        {
            "role": "user",
            "content": [
                {"type": "text", "text": "What should the robot do next?"},
                {"type": "image_url", "image_url": {"url": image_url}}
            ]
        }
    ],
    max_tokens=300
)

action_plan = response.choices[0].message.content
```

### LLaVA: Open-Source Vision-Language Model

```python
from llava.model import LlavaLlamaForCausalLM
from llava.conversation import conv_templates

model = LlavaLlamaForCausalLM.from_pretrained("liuhaotian/llava-v1.5-7b")

# Process image + text
response = model.generate(
    image=image,
    prompt="Describe what the robot should do to clean this room.",
    max_tokens=200
)
```

## Hands-On Projects

### Project 1: CLIP for Object Detection
- Set up CLIP in ROS 2
- Classify objects from camera feed
- Test with novel object categories

### Project 2: VQA for Scene Understanding
- Deploy BLIP-2 or similar VQA model
- Answer questions about robot's environment
- Use for task verification

### Project 3: Grounding DINO for Manipulation
- Detect objects from text descriptions
- Convert 2D detections to 3D poses
- Integrate with grasp planner

### Project 4: Multimodal LLM Integration
- Send camera images to GPT-4 Vision
- Get action plans from visual observations
- Compare with traditional CV pipelines

## Performance Considerations

### Model Size vs Speed

| Model | Size | Speed (FPS) | Accuracy |
|-------|------|-------------|----------|
| CLIP ViT-B/32 | 150MB | 30+ | Good |
| CLIP ViT-L/14 | 900MB | 10 | Excellent |
| Grounding DINO | 600MB | 5 | Very Good |
| BLIP-2 | 3GB | 2 | Excellent |
| GPT-4V | API | 1-2s latency | Best |

### Optimization

1. **Use smaller models** for real-time (CLIP-ViT-B)
2. **GPU acceleration** essential for VLMs
3. **Batch processing** when possible
4. **Cache embeddings** for repeated queries
5. **Hybrid approach**: Fast CV for tracking, VLM for queries

## Best Practices

1. **Validate detections**: VLMs can hallucinate
2. **Confidence thresholds**: Filter low-confidence outputs
3. **Combine with classical CV**: Use VLMs for high-level, CV for low-level
4. **Test edge cases**: Unusual lighting, occlusion, novel objects
5. **Monitor latency**: Real-time constraints matter

*For now, explore [OpenCLIP](https://github.com/mlfoundations/open_clip), [Grounding DINO](https://github.com/IDEA-Research/GroundingDINO), and [BLIP-2](https://github.com/salesforce/LAVIS).*
