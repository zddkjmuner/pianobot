from __future__ import annotations

from pianobot.audio import score_onsets
from pianobot.scores import (
    generate_note_from_chord,
    generate_salut_damour_high_note_score,
    generate_salut_damour_low_chord_score,
)


def expanded_note_count(sequence):
    scheduled, _ = score_onsets(sequence)
    count = 0
    for _, event in scheduled:
        count += len(generate_note_from_chord(event.name, event.key_scale))
    return len(scheduled), count


def test_salut_damour_low_score_counts_match_notebook() -> None:
    sequence = generate_salut_damour_low_chord_score()
    scheduled_count, note_count = expanded_note_count(sequence)
    assert scheduled_count == 53
    assert note_count == 103


def test_salut_damour_high_score_counts_match_notebook() -> None:
    sequence = generate_salut_damour_high_note_score()
    scheduled_count, note_count = expanded_note_count(sequence)
    assert scheduled_count == 40
    assert note_count == 46


def test_score_onset_end_times_match_notebook_logs() -> None:
    low_end = score_onsets(generate_salut_damour_low_chord_score())[1]
    high_end = score_onsets(generate_salut_damour_high_note_score())[1]
    assert abs(low_end - 38.09) < 0.02
    assert abs(high_end - 37.85) < 0.02
