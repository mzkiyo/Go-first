package main

import (
	"archive/zip"
	"bytes"
	"encoding/json"
	"fmt"
	"io"
	"log"
	"math"
	"net/http"
	"os"
	"path/filepath"
	"strconv"
	"time"

	"github.com/gin-gonic/gin"
	"gocv.io/x/gocv"
)

type KeyframeMove struct {
	Time   float64 `json:"time"`
	Value  struct {
		PosX    float64 `json:"posX"`
		PosY    float64 `json:"posY"`
		PosZ    float64 `json:"posZ"`
		AnchorX float64 `json:"anchorX"`
		AnchorY float64 `json:"anchorY"`
		AnchorZ float64 `json:"anchorZ"`
	} `json:"value"`
	Easing []float64 `json:"easing"`
}

type KeyframeRotate struct {
	Time  float64 `json:"time"`
	Value struct {
		RotZ float64 `json:"rotZ"`
		RotX float64 `json:"rotX"`
		RotY float64 `json:"rotY"`
	} `json:"value"`
	Easing []float64 `json:"easing"`
}

type TrackPoint struct {
	T   float64
	Dx  float64
	Dy  float64
	Rot float64
	Pts int
}

func kalmanFilter(data []float64, Q, R float64) []float64 {
	if len(data) == 0 {
		return data
	}
	out := make([]float64, len(data))
	x := data[0]
	P := 1.0
	out[0] = x
	for i := 1; i < len(data); i++ {
		P += Q
		K := P / (P + R)
		x = x + K*(data[i]-x)
		P = (1 - K) * P
		out[i] = x
	}
	return out
}

func detectGila1000(gray gocv.Mat) []gocv.Point2f {
	var allPoints []gocv.Point2f
	cols, rows := 8, 6
	cw := gray.Cols() / cols
	ch := gray.Rows() / rows

	for r := 0; r < rows; r++ {
		for c := 0; c < cols; c++ {
			rect := gocv.NewRect(c*cw, r*ch, cw, ch)
			roi := gray.Region(rect)
			corners := gocv.NewMat()
			gocv.GoodFeaturesToTrackWithParams(roi, &corners, 30, 0.003, 3, gocv.NewMat(), 3, false, 0.04)
			for i := 0; i < corners.Rows(); i++ {
				pt := corners.GetVecfAt(i, 0)
				if len(pt) >= 2 {
					allPoints = append(allPoints, gocv.Point2f{X: pt[0] + float32(c*cw), Y: pt[1] + float32(r*ch)})
				}
			}
			roi.Close()
			corners.Close()
		}
	}
	if len(allPoints) < 150 {
		corners := gocv.NewMat()
		gocv.GoodFeaturesToTrackWithParams(gray, &corners, 1000, 0.001, 2, gocv.NewMat(), 3, false, 0.04)
		for i := 0; i < corners.Rows(); i++ {
			pt := corners.GetVecfAt(i, 0)
			if len(pt) >= 2 {
				allPoints = append(allPoints, gocv.Point2f{X: pt[0], Y: pt[1]})
			}
		}
		corners.Close()
	}
	// cornerSubPix for refinement
	if len(allPoints) > 0 {
		ptsMat := gocv.NewMatWithSize(len(allPoints), 1, gocv.MatTypeCV32FC2)
		for i, p := range allPoints {
			ptsMat.SetFloatAt(i, 0, p.X)
			ptsMat.SetFloatAt(i, 1, p.Y)
		}
		term := gocv.NewTermCriteria(gocv.TermCriteriaEPS+gocv.TermCriteriaMaxIter, 30, 0.01)
		gocv.CornerSubPix(gray, ptsMat, gocv.NewSize(5, 5), gocv.NewSize(-1, -1), term)
		// read back
		for i := 0; i < ptsMat.Rows(); i++ {
			allPoints[i] = gocv.Point2f{X: ptsMat.GetFloatAt(i, 0), Y: ptsMat.GetFloatAt(i, 1)}
		}
		ptsMat.Close()
	}
	return allPoints
}

func trackVideo(videoPath string, sampleFPS float64) ([]TrackPoint, float64, error) {
	cap, err := gocv.OpenVideoCapture(videoPath)
	if err != nil {
		return nil, 0, err
	}
	defer cap.Close()

	fps := cap.Get(gocv.VideoCaptureFPS)
	frameCount := cap.Get(gocv.VideoCaptureFrameCount)
	duration := frameCount / fps
	if fps == 0 {
		fps = 30
		duration = 5
	}
	totalFrames := int(duration * sampleFPS)
	log.Printf("Tracking: fps=%.2f duration=%.2f totalSamples=%d", fps, duration, totalFrames)

	var path []TrackPoint
	var prevGray gocv.Mat
	var prevPts []gocv.Point2f
	var accumDx, accumDy, accumRot float64

	for i := 0; i < totalFrames; i++ {
		t := float64(i) / sampleFPS
		framePos := int((t / duration) * frameCount)
		cap.Set(gocv.VideoCapturePosFrames, float64(framePos))

		frame := gocv.NewMat()
		if !cap.Read(&frame) || frame.Empty() {
			frame.Close()
			continue
		}

		// resize to 640x360 for speed (max hasil di 3GB)
		resized := gocv.NewMat()
		gocv.Resize(frame, &resized, gocv.NewSize(640, 360), 0, 0, gocv.InterpolationLinear)
		frame.Close()

		gray := gocv.NewMat()
		gocv.CvtColor(resized, &gray, gocv.ColorBGRToGray)
		gocv.EqualizeHist(gray, &gray)
		blurred := gocv.NewMat()
		gocv.GaussianBlur(gray, &blurred, gocv.NewSize(3, 3), 0, 0, gocv.BorderDefault)

		gray.Close()
		resized.Close()

		if prevGray.Empty() || len(prevPts) < 50 {
			if !prevGray.Empty() {
				prevGray.Close()
			}
			prevGray = blurred.Clone()
			prevPts = detectGila1000(prevGray)
			if len(path) == 0 {
				path = append(path, TrackPoint{T: t, Dx: 0, Dy: 0, Rot: 0, Pts: len(prevPts)})
			}
			log.Printf("Frame %d: detected %d points", i, len(prevPts))
			blurred.Close()
		} else {
			nextPts := make([]gocv.Point2f, len(prevPts))
			status := make([]uint8, len(prevPts))
			errMat := make([]float32, len(prevPts))

			// calcOpticalFlowPyrLK
			// gocv.CalcOpticalFlowPyrLK(prevGray, blurred, prevPts, nextPts, status, errMat)
			// Use mat version for GoCV
			prevMat := gocv.NewMatWithSize(len(prevPts), 1, gocv.MatTypeCV32FC2)
			for j, p := range prevPts {
				prevMat.SetFloatAt(j, 0, p.X)
				prevMat.SetFloatAt(j, 1, p.Y)
			}
			nextMat := gocv.NewMat()
			statusMat := gocv.NewMat()
			errMatGocv := gocv.NewMat()
			gocv.CalcOpticalFlowPyrLKWithParams(prevGray, blurred, prevMat, nextMat, statusMat, errMatGocv, gocv.NewSize(21, 21), 4, gocv.NewTermCriteria(gocv.TermCriteriaEPS+gocv.TermCriteriaMaxIter, 30, 0.01), 0, 0.001)

			var goodPrev []gocv.Point2f
			var goodNext []gocv.Point2f
			for j := 0; j < statusMat.Rows(); j++ {
				if statusMat.GetUCharAt(j, 0) == 1 {
					e := errMatGocv.GetFloatAt(j, 0)
					if e < 18 {
						goodPrev = append(goodPrev, prevPts[j])
						goodNext = append(goodNext, gocv.Point2f{X: nextMat.GetFloatAt(j, 0), Y: nextMat.GetFloatAt(j, 1)})
					}
				}
			}

			var dx, dy, rot float64
			if len(goodPrev) > 40 {
				prevPtsMat := gocv.NewMatWithSize(len(goodPrev), 1, gocv.MatTypeCV32FC2)
				nextPtsMat := gocv.NewMatWithSize(len(goodNext), 1, gocv.MatTypeCV32FC2)
				for j := range goodPrev {
					prevPtsMat.SetFloatAt(j, 0, goodPrev[j].X)
					prevPtsMat.SetFloatAt(j, 1, goodPrev[j].Y)
					nextPtsMat.SetFloatAt(j, 0, goodNext[j].X)
					nextPtsMat.SetFloatAt(j, 1, goodNext[j].Y)
				}
				H := gocv.FindHomographyWithParams(prevPtsMat, nextPtsMat, gocv.HomographyMethodRANSAC, 1.5, gocv.NewMat(), 2000, 0.995)
				if !H.Empty() && H.Rows() == 3 && H.Cols() == 3 {
					dx = H.GetDoubleAt(0, 2)
					dy = H.GetDoubleAt(1, 2)
					rot = math.Atan2(H.GetDoubleAt(1, 0), H.GetDoubleAt(0, 0)) * 180 / math.Pi
					if math.Abs(dx) > 80 {
						dx *= 0.3
					}
					if math.Abs(dy) > 80 {
						dy *= 0.3
					}
					if math.Abs(rot) > 12 {
						rot *= 0.2
					}
				}
				prevPtsMat.Close()
				nextPtsMat.Close()
				H.Close()
			}
			accumDx += dx
			accumDy += dy
			accumRot += rot
			path = append(path, TrackPoint{T: t, Dx: accumDx, Dy: accumDy, Rot: accumRot, Pts: len(goodPrev)})

			prevMat.Close()
			nextMat.Close()
			statusMat.Close()
			errMatGocv.Close()
			prevGray.Close()
			prevGray = blurred.Clone()
			prevPts = goodNext
			blurred.Close()
		}
	}

	if !prevGray.Empty() {
		prevGray.Close()
	}
	return path, duration, nil
}

func generateOFTS(path []TrackPoint, duration float64) ([]byte, error) {
	// kalman
	var xs, ys, rs []float64
	for _, p := range path {
		xs = append(xs, p.Dx)
		ys = append(ys, p.Dy)
		rs = append(rs, p.Rot)
	}
	xs = kalmanFilter(xs, 0.003, 0.4)
	ys = kalmanFilter(ys, 0.003, 0.4)
	rs = kalmanFilter(rs, 0.01, 0.3)

	var moveKFs []KeyframeMove
	var rotKFs []KeyframeRotate
	for i, p := range path {
		m := KeyframeMove{Time: math.Round(p.T*10000) / 10000, Easing: []float64{0.85, 0, 0.15, 1}}
		m.Value.PosX = 960 - xs[i]*1.8
		m.Value.PosY = 540 - ys[i]*1.8
		m.Value.PosZ = math.Sin(p.T*0.3) * 50
		moveKFs = append(moveKFs, m)

		r := KeyframeRotate{Time: math.Round(p.T*10000) / 10000, Easing: []float64{0.85, 0, 0.15, 1}}
		r.Value.RotZ = rs[i]
		r.Value.RotX = 0
		r.Value.RotY = rs[i] * 0.6
		rotKFs = append(rotKFs, r)
	}

	project := map[string]interface{}{
		"id":           fmt.Sprintf("prj_%d", time.Now().UnixMilli()),
		"name":         "Tracked_Cam_1000pts_Golang",
		"aspectRatio":  "16:9",
		"resolution":   "1080p",
		"fps":          "60",
		"bgColor":      "transparent",
		"createdAt":    time.Now().Format(time.RFC3339),
		"updatedAt":    time.Now().Format(time.RFC3339),
		"sizeBytes":    0,
		"beatmarks":    []interface{}{},
		"markerNames":  map[string]interface{}{},
		"pixelsPerSecond": 57,
		"duration":     duration,
		"motionBlur":   map[string]interface{}{"enabled": false, "shutterAngle": 180, "shutterPhase": 0, "samples": 16},
		"isTemplate":   false,
		"thumbnail":    "",
		"previewUrl":   "",
		"media":        []interface{}{},
		"layers": []map[string]interface{}{
			{
				"id": "layer_null_tracked_golang", "name": "Tracked_Cam_1000pts", "type": "null", "parentId": nil,
				"startPx": 0, "startSec": 0, "durationSec": duration, "widthPx": duration * 57,
				"sourceOffsetSec": 0, "mediaDuration": nil, "isDurationExplicit": false, "isMuted": false,
				"normX": 0.44, "normY": 0.4677, "posX": moveKFs[0].Value.PosX, "posY": moveKFs[0].Value.PosY, "posZ": 0,
				"anchorX": 0, "anchorY": 0, "anchorZ": 0, "scaleW": 100, "scaleH": 100, "scaleZ": 100,
				"rotation": 0, "rotX": 0, "rotY": 0, "rotZ": 0, "skewX": 0, "skewY": 0, "opacity": 1, "blendMode": "normal",
				"hasBrightnessContrast": false, "brightness": 0, "contrast": 0, "effectsDisabled": false,
				"thumbUrl": "", "dataUrl": "", "mediaWidth": nil, "mediaHeight": nil,
				"_userResized": false, "userTrimmed": false, "hidden": false, "motionBlur": false,
				"is3D": true, "collapseTransformations": false,
				"keyframes": map[string]interface{}{"move": moveKFs, "rotate": rotKFs},
				"isSolid": false, "transformScaleX": 1, "transformScaleY": 1, "cameraBlurEnabled": false,
				"volume": 1, "speed": 1, "speedInterpolation": "none", "preservePitch": true,
			},
		},
	}

	jsonData, _ := json.Marshal(project)
	buf := new(bytes.Buffer)
	zw := zip.NewWriter(buf)
	f, _ := zw.Create("project.json")
	f.Write(jsonData)
	zw.Close()
	return buf.Bytes(), nil
}

func generateKF(path []TrackPoint, duration float64) ([]byte, error) {
	var xs, ys, rs []float64
	for _, p := range path {
		xs = append(xs, p.Dx)
		ys = append(ys, p.Dy)
		rs = append(rs, p.Rot)
	}
	xs = kalmanFilter(xs, 0.003, 0.4)
	ys = kalmanFilter(ys, 0.003, 0.4)
	rs = kalmanFilter(rs, 0.01, 0.3)

	id := "null_tracked_golang_001"
	kfPosX := []map[string]interface{}{}
	kfPosY := []map[string]interface{}{}
	kfRotateY := []map[string]interface{}{}
	kfRotateX := []map[string]interface{}{}
	kfRotate := []map[string]interface{}{}
	kfScale := []map[string]interface{}{}

	for i, p := range path {
		tMs := int(p.T * 1000)
		kfPosX = append(kfPosX, map[string]interface{}{"t": tMs, "v": 960 - xs[i]*1.8, "i": "LINEAR"})
		kfPosY = append(kfPosY, map[string]interface{}{"t": tMs, "v": 540 - ys[i]*1.8, "i": "LINEAR"})
		kfRotateY = append(kfPosY, map[string]interface{}{"t": tMs, "v": rs[i] * 0.6, "i": "LINEAR"})
		kfRotateX = append(kfPosX, map[string]interface{}{"t": tMs, "v": 0, "i": "LINEAR"})
		kfRotate = append(kfPosY, map[string]interface{}{"t": tMs, "v": rs[i], "i": "LINEAR"})
		kfScale = append(kfPosX, map[string]interface{}{"t": tMs, "v": 100, "i": "LINEAR"})
	}

	project := map[string]interface{}{
		"layers": []map[string]interface{}{
			{"id": id, "name": "Tracked_Cam_1000pts", "type": "NULL", "isLocked": false, "inPointMs": 0, "outPointMs": int(duration * 1000), "content": map[string]string{"kind": "NULL"}, "is3D": true},
		},
		"transformX": map[string]string{id: fmt.Sprintf("%.2f", 960.0)},
		"transformY": map[string]string{id: fmt.Sprintf("%.2f", 540.0)},
		"scale":      map[string]string{id: "100"},
		"rotate":     map[string]string{id: "0"},
		"anchorX":    map[string]string{id: "0.5"},
		"anchorY":    map[string]string{id: "0.5"},
		"opacity":    map[string]string{id: "1.0"},
		"visibility": map[string]string{id: "true"},
		"kfPosX":     map[string]interface{}{id: kfPosX},
		"kfPosY":     map[string]interface{}{id: kfPosY},
		"kfRotateY":  map[string]interface{}{id: kfRotateY},
		"kfRotateX":  map[string]interface{}{id: kfRotateX},
		"kfRotate":   map[string]interface{}{id: kfRotate},
		"kfScale":    map[string]interface{}{id: kfScale},
		"kfPosZ":     map[string]interface{}{},
		"kfScaleY":   map[string]interface{}{},
		"kfOpacity":  map[string]interface{}{},
		"backgroundColor": 4279900701,
		"shutterAngle":    180,
	}

	manifest := map[string]interface{}{
		"format": 1, "exportedAt": time.Now().UnixMilli(), "app": "1.2.4", "build": 26,
		"includesMedia": true,
		"project": map[string]interface{}{"name": "Tracked_Golang", "fps": 30, "ratio": "16:9", "resolution": "1080p", "createdAt": time.Now().UnixMilli()},
		"assets": map[string]interface{}{},
	}

	buf := new(bytes.Buffer)
	zw := zip.NewWriter(buf)
	f1, _ := zw.Create("manifest.json")
	mj, _ := json.Marshal(manifest)
	f1.Write(mj)
	f2, _ := zw.Create("project.json")
	pj, _ := json.Marshal(project)
	f2.Write(pj)
	zw.Close()
	return buf.Bytes(), nil
}

func main() {
	gin.SetMode(gin.ReleaseMode)
	r := gin.Default()
	r.Static("/static", "./static")
	r.LoadHTMLGlob("templates/*")

	r.GET("/", func(c *gin.Context) {
		c.HTML(200, "index.html", nil)
	})

	r.POST("/api/track", func(c *gin.Context) {
		file, err := c.FormFile("video")
		if err != nil {
			c.JSON(400, gin.H{"error": "no video"})
			return
		}
		sampleFPS, _ := strconv.ParseFloat(c.DefaultPostForm("sampleFPS", "5"), 64)

		tmpPath := filepath.Join(os.TempDir(), fmt.Sprintf("upload_%d_%s", time.Now().Unix(), file.Filename))
		c.SaveUploadedFile(file, tmpPath)
		defer os.Remove(tmpPath)

		log.Printf("Tracking %s sampleFPS=%.1f", tmpPath, sampleFPS)
		path, duration, err := trackVideo(tmpPath, sampleFPS)
		if err != nil {
			c.JSON(500, gin.H{"error": err.Error()})
			return
		}
		if len(path) < 2 {
			c.JSON(500, gin.H{"error": "tracking failed, path empty"})
			return
		}

		// save tracking result to temp json for download
		id := fmt.Sprintf("%d", time.Now().Unix())
		trackData, _ := json.Marshal(path)
		os.WriteFile(filepath.Join(os.TempDir(), "track_"+id+".json"), trackData, 0644)

		c.JSON(200, gin.H{
			"id":       id,
			"duration": duration,
			"points":   len(path),
			"firstPts": path[0].Pts,
			"lastPts":  path[len(path)-1].Pts,
		})
	})

	r.GET("/api/download/ofts/:id", func(c *gin.Context) {
		id := c.Param("id")
		data, err := os.ReadFile(filepath.Join(os.TempDir(), "track_"+id+".json"))
		if err != nil {
			c.JSON(404, gin.H{"error": "track not found"})
			return
		}
		var path []TrackPoint
		json.Unmarshal(data, &path)
		// get duration from path
		duration := 5.0
		if len(path) > 0 {
			duration = path[len(path)-1].T + 0.5
		}
		ofts, _ := generateOFTS(path, duration)
		c.Header("Content-Disposition", "attachment; filename=Tracked_1000pts.ofts")
		c.Data(200, "application/octet-stream", ofts)
	})

	r.GET("/api/download/kf/:id", func(c *gin.Context) {
		id := c.Param("id")
		data, err := os.ReadFile(filepath.Join(os.TempDir(), "track_"+id+".json"))
		if err != nil {
			c.JSON(404, gin.H{"error": "track not found"})
			return
		}
		var path []TrackPoint
		json.Unmarshal(data, &path)
		duration := 5.0
		if len(path) > 0 {
			duration = path[len(path)-1].T + 0.5
		}
		kf, _ := generateKF(path, duration)
		c.Header("Content-Disposition", "attachment; filename=Tracked_1000pts.kf")
		c.Data(200, "application/octet-stream", kf)
	})

	port := os.Getenv("SERVER_PORT")
	if port == "" {
		port = "8080"
	}
	// for pterodactyl
	if p := os.Getenv("PORT"); p != "" {
		port = p
	}
	log.Printf("FishTracker Golang Server running on :%s | 3GB optimized | 1000 titik", port)
	r.Run("0.0.0.0:" + port)
}

func readAll(r io.Reader) ([]byte, error) {
	return io.ReadAll(r)
}
