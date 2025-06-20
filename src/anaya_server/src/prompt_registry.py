from src.prompts import (
    base_go_here_orientation_prompt,
    base_go_here_strict_orientation_prompt,
    cot_few_shot_go_here_orientation_prompt,
    cot_few_shot_go_here_strict_orientation_prompt,
    cot_go_here_orientation_prompt,
    cot_go_here_strict_orientation_prompt,
    few_shot_go_here_orientation_prompt,
    few_shot_go_here_strict_orientation_prompt,
    dummy_prompt,
)

prompt_registry = {
    "fix": {
        "base_go_here_strict_orientation_prompt": base_go_here_strict_orientation_prompt,
        "cot_go_here_strict_orientation_prompt": cot_go_here_strict_orientation_prompt,
        "few_shot_go_here_strict_orientation_prompt": few_shot_go_here_strict_orientation_prompt,
        "cot_few_shot_go_here_strict_orientation_prompt": cot_few_shot_go_here_strict_orientation_prompt,
    },
    "free": {
        "base_go_here_orientation_prompt": base_go_here_orientation_prompt,
        "cot_go_here_orientation_prompt": cot_go_here_orientation_prompt,
        "few_shot_go_here_orientation_prompt": few_shot_go_here_orientation_prompt,
        "cot_few_shot_go_here_orientation_prompt": cot_few_shot_go_here_orientation_prompt,
    },
    # Dummy mode and prompt for testing
    "dummy_mode": {"dummy_prompt": dummy_prompt},
}
