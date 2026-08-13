import torch
import transformers
import trl
import os
import torch
from transformers import AutoModelForImageTextToText, AutoProcessor
import wandb
from datetime import datetime

CONFIG = {
    # Model settings
    "model_id": "LiquidAI/LFM2.5-VL-1.6B",
    "max_image_tokens": 256,
    "torch_dtype": "bfloat16",

    # Dataset settings
    "dataset_name": "VishwanathAS/carla_lane_following",
    "test_split_ratio": 0.2,
    "dataset_seed": 42,

    # Training settings
    "output_dir": "./lfm2-vl-carla-finetuned",
    "num_train_epochs": 1,
    "per_device_train_batch_size": 1,
    "per_device_eval_batch_size": 1,
    "gradient_accumulation_steps": 16,
    "learning_rate": 5e-4,
    "warmup_ratio": 0.1,
    "weight_decay": 0.01,
    "logging_steps": 10,
    "save_steps": 200,
    "save_total_limit": 3,
    "eval_steps": 200,
    "max_length": 512,
    "gradient_checkpointing": True,
    "optim": "adamw_torch_8bit",

    # LoRA settings (set use_lora=False for full fine-tuning)
    "use_lora": True,
    "lora_r": 8,
    "lora_alpha": 16,
    "lora_dropout": 0.05,
    "lora_target_modules": [
        "q_proj", "v_proj", "fc1", "fc2", "linear",
        "gate_proj", "up_proj", "down_proj",
    ],

    # W&B settings
    "wandb_project": "lfm2-vl-carla",
    "wandb_run_name": None,

    # Resume training
    "resume_from_checkpoint": None,  # Path to checkpoint or "latest"
}

print(f"📦 PyTorch version: {torch.__version__}")
print(f"🤗 Transformers version: {transformers.__version__}")
print(f"📊 TRL version: {trl.__version__}")


run_name = CONFIG["wandb_run_name"] or f"lfm2-vl-{datetime.now().strftime('%Y%m%d_%H%M%S')}"
wandb.init(
        project=CONFIG["wandb_project"],
        name=run_name,
        config=CONFIG,
        resume="allow",
    )

print("Wandb initialized with run name:", run_name)
model_id = CONFIG["model_id"]

print("📚 Loading processor...")
processor_source = model_id
processor = AutoProcessor.from_pretrained(
    processor_source,
    max_image_tokens=CONFIG["max_image_tokens"],
    trust_remote_code=True
)

print("🧠 Loading model...")
model = AutoModelForImageTextToText.from_pretrained(
    model_id,
    torch_dtype=CONFIG["torch_dtype"],
    device_map="auto",
    trust_remote_code=True
)

print("\n✅ Local model loaded successfully!")
print(f"📖 Vocab size: {len(processor.tokenizer)}")
print(f"🔢 Parameters: {model.num_parameters():,}")
print(f"💾 Model size: ~{model.num_parameters() * 2 / 1e9:.1f} GB ({CONFIG['torch_dtype']})")


#prepare dataset
from datasets import load_dataset

raw_ds = load_dataset(CONFIG["dataset_name"])
full_dataset = raw_ds["train"]
split = full_dataset.train_test_split(test_size=CONFIG["test_split_ratio"], seed=CONFIG["dataset_seed"])
train_dataset = split["train"]
eval_dataset = split["test"]

print("✅ SFT Dataset loaded:")
print(f"   📚 Train samples: {len(train_dataset)}")
print(f"   🧪 Eval samples: {len(eval_dataset)}")
print(f"\n📝 Single Sample: [IMAGE] {train_dataset[0]['image']} {train_dataset[0]['instruction']}")

#conversation format
def format_medical_sample(sample):
    return [
        # {"role": "system", "content": [{"type": "text", "text": system_message}]},
        {
            "role": "user",
            "content": [
                {"type": "image", "image": sample["image"]},
                {"type": "text", "text": sample["instruction"]},
            ],
        },
        {"role": "assistant", "content": [{"type": "text", "text": sample["trajectory"]}]},
    ]

train_dataset = train_dataset.map(lambda s: {"messages": format_medical_sample(s)}, remove_columns=train_dataset.column_names)
eval_dataset = eval_dataset.map(lambda s: {"messages": format_medical_sample(s)}, remove_columns=eval_dataset.column_names)

print("✅ SFT Dataset formatted:")
print(f"   📚 Train samples: {len(train_dataset)}")
print(f"   🧪 Eval samples: {len(eval_dataset)}")


#format images to RGB
from PIL import Image
def ensure_rgb_images(sample):
    """Converts all images in a sample to RGB."""
    for message in sample["messages"]:
        if isinstance(message.get("content"), list):
            for content_part in message["content"]:
                if content_part.get("type") == "image":
                    img = content_part["image"]
                    if img.mode != "RGB":
                        content_part["image"] = img.convert("RGB")
    return sample

train_dataset = train_dataset.map(ensure_rgb_images)
eval_dataset = eval_dataset.map(ensure_rgb_images)
print("✅ Processed datasets: Converted images to RGB.")

#prepare collate
def create_collate_fn(processor):
    """Create a collate function that prepares batch inputs for the processor."""
    def collate_fn(sample):
        batch = processor.apply_chat_template(sample, tokenize=True, return_dict=True, return_tensors="pt")
        labels = batch["input_ids"].clone()
        labels[labels == processor.tokenizer.pad_token_id] = -100
        batch["labels"] = labels
        return batch
    return collate_fn

collate_fn = create_collate_fn(processor)

from peft import LoraConfig, get_peft_model

target_modules = [
    "q_proj", "v_proj", "fc1", "fc2", "linear",
    "gate_proj", "up_proj", "down_proj",
]

peft_config = LoraConfig(
    lora_alpha=16,
    lora_dropout=0.05,
    r=8,
    bias="none",
    target_modules=target_modules,
    task_type="CAUSAL_LM",
)

model = get_peft_model(model, peft_config)
model.print_trainable_parameters()

from trl import SFTConfig, SFTTrainer

sft_config = SFTConfig(
    output_dir="lfm2-vl-carla",
    num_train_epochs=1,
    per_device_train_batch_size=1,
    gradient_accumulation_steps=16,
    learning_rate=5e-4,
    warmup_ratio=0.1,
    weight_decay=0.01,
    logging_steps=10,
    optim="adamw_torch_8bit",
    gradient_checkpointing=True,
    max_length=512,
    dataset_kwargs={"skip_prepare_dataset": True},
)

print("🏗️  Creating SFT trainer...")
sft_trainer = SFTTrainer(
    model=model,
    args=sft_config,
    train_dataset=train_dataset,
    eval_dataset=eval_dataset,
    data_collator=collate_fn,
    processing_class=processor.tokenizer,
)

print("\n🚀 Starting SFT training...")
sft_trainer.train()

print("🎉 SFT training completed!")

sft_trainer.save_model()
print(f"💾 Saving to: {sft_config.output_dir}")

model = model.merge_and_unload()
model.save_pretrained(sft_config.output_dir)
processor.save_pretrained(sft_config.output_dir)
print("✅ Model and processor saved successfully.")

wandb.finish()