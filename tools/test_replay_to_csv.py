import csv, json, math, os, sys, tempfile, unittest
sys.path.insert(0, os.path.dirname(__file__))
import replay_to_csv as m


def frame(t):
    h = math.pi / 4
    q = [math.cos(h / 2), 0, 0, math.sin(h / 2)]  # yaw = 45 deg
    return {"time": t, "ball": {"position": [t, 1.0, 0.1]},
            "robots": [{"name": f"robot{i}", "basePosition": [i, -i, 0.5], "baseQuaternion": q,
                        "gameControlPosition": [0, 0, 0], "gameControlYaw": 0.0,
                        "joints": {"AAHead_yaw": 0.3, "Head_pitch": -0.1}} for i in range(1, 7)]}


class T(unittest.TestCase):
    def test_roundtrip(self):
        with tempfile.TemporaryDirectory() as d:
            os.makedirs(f"{d}/chunks")
            json.dump({"hz": 10}, open(f"{d}/manifest.json", "w"))
            with open(f"{d}/chunks/frames_000000.jsonl", "w") as fh:
                for k in range(0, 11):
                    fh.write(json.dumps(frame(k * 0.1)) + "\n")
            out = f"{d}/o.csv"
            m.main([d, out, "--dt", "0.05"])
            rows = list(csv.DictReader(open(out)))
            self.assertEqual(len(rows), 21 * 7)
            r = [x for x in rows if x["id"] == "4" and abs(float(x["t"]) - 0.5) < 1e-6][0]
            self.assertEqual(r["team"], "2")
            self.assertAlmostEqual(float(r["theta"]), math.pi / 4, 4)
            b = [x for x in rows if x["id"] == "0" and abs(float(x["t"]) - 0.25) < 1e-6][0]
            self.assertAlmostEqual(float(b["x"]), 0.25, 4)

            self.assertEqual(list(rows[0].keys()),
                             ["t", "id", "team", "x", "y", "theta", "z", "roll", "pitch", "headYaw", "headPitch"])
            self.assertAlmostEqual(float(r["z"]), 0.5, 4)
            self.assertAlmostEqual(float(r["roll"]), 0.0, 4)
            self.assertAlmostEqual(float(r["pitch"]), 0.0, 4)
            self.assertAlmostEqual(float(r["headYaw"]), 0.3, 4)
            self.assertAlmostEqual(float(r["headPitch"]), -0.1, 4)
            self.assertEqual(float(b["headYaw"]), 0.0)

    def test_tilt(self):
        h = 0.2  # pure pitch 0.2 about y, then pure roll 0.1 about x
        self.assertAlmostEqual(m.quat_roll_pitch([math.cos(h / 2), 0, math.sin(h / 2), 0])[1], 0.2, 6)
        h = 0.1
        self.assertAlmostEqual(m.quat_roll_pitch([math.cos(h / 2), math.sin(h / 2), 0, 0])[0], 0.1, 6)


if __name__ == "__main__":
    unittest.main()
