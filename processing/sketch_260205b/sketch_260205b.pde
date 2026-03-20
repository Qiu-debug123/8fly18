import processing.serial.*;

Serial myPort;
String data = "";

// --- 用户可修改的配置区域 ---
float xRangeMin = 0;   // X轴最小值
float xRangeMax = 5;   // X轴最大值
float yRangeMin = 0;   // Y轴最小值
float yRangeMax = 5;   // Y轴最大值
String serialPortName = "COM13"; // 串口号
int baudRate = 115200;           // 波特率 (通常单片机是115200，如有不同请修改)
// ----------------------------

float currentX = 0;
float currentY = 0;

// Set B 坐标 (例如: 融合后结果)
float currentBX = 0;
float currentBY = 0;

// 存储历史轨迹点
ArrayList<PVector> trail = new ArrayList<PVector>();
// 存储历史轨迹点 Set B
ArrayList<PVector> trailB = new ArrayList<PVector>();

void setup() {
  size(800, 800); // 设置窗口大小
  
  // 初始化串口
  try {
    myPort = new Serial(this, serialPortName, baudRate);
    myPort.bufferUntil('\n'); // 读到换行符才产生 serialEvent
  } catch (Exception e) {
    println("Error: 串口打开失败，请检查端口号是否正确。");
  }
  
  textSize(16);
}

void draw() {
  background(255); // 每帧清空背景，避免文字重叠
  
  // 定义绘图边距，给坐标轴文字留空间
  float margin = 60;
  float plotWidth = width - 2 * margin;
  float plotHeight = height - 2 * margin;
  
  // 移动原点到绘图区的左下角 (Processing默认原点在左上角)
  translate(margin, height - margin);
  
  // --- 绘制坐标轴 ---
  stroke(0);
  strokeWeight(2);
  
  // X轴
  line(0, 0, plotWidth, 0); 
  // Y轴 (注意：在translate后，y轴向上是负方向)
  line(0, 0, 0, -plotHeight); 
  
  fill(0);
  textAlign(CENTER);
  
  // 绘制 X 轴刻度和标签
  int xDivisions = 5; // X轴切分份数
  for (int i = 0; i <= xDivisions; i++) {
    float xPos = map(i, 0, xDivisions, 0, plotWidth);
    line(xPos, 0, xPos, 5); // 刻度线
    float val = map(i, 0, xDivisions, xRangeMin, xRangeMax);
    text(nf(val, 0, 1), xPos, 25); // 标签数值
  }
  text("X Axis", plotWidth / 2, 45); // X轴标题
  
  // 绘制 Y 轴刻度和标签
  textAlign(RIGHT, CENTER);
  int yDivisions = 5; // Y轴切分份数
  for (int i = 0; i <= yDivisions; i++) {
    float yPos = map(i, 0, yDivisions, 0, -plotHeight);
    line(0, yPos, -5, yPos); // 刻度线
    float val = map(i, 0, yDivisions, yRangeMin, yRangeMax);
    text(nf(val, 0, 1), -10, yPos); // 标签数值
  }
  
  // 保存当前的变换状态
  pushMatrix(); 
  rotate(radians(-90));
  textAlign(CENTER);
  text("Y Axis", plotHeight/2, -40); // Y轴标题
  popMatrix();
  
    // --- 绘制历史轨迹 (Set B - 绿色) ---
  noFill();
  stroke(0, 200, 0); // 绿色轨迹 (略深一点的绿)
  strokeWeight(1.5); // 细线
  
  beginShape();
  synchronized(trailB) {
    for (PVector p : trailB) {
      // 限制在范围内
      float drawTx = constrain(p.x, xRangeMin, xRangeMax);
      float drawTy = constrain(p.y, yRangeMin, yRangeMax);
      
      float sx = map(drawTx, xRangeMin, xRangeMax, 0, plotWidth);
      float sy = map(drawTy, yRangeMin, yRangeMax, 0, -plotHeight);
      vertex(sx, sy);
    }
  }
  endShape();
  
  // --- 绘制历史轨迹 (Set A - 蓝色) ---
  noFill();
  stroke(0, 0, 255); // 蓝色轨迹
  strokeWeight(1.5); // 细线
  
  beginShape();
  // 使用 synchronized 防止多线程修改导致崩溃
  synchronized(trail) {
    for (PVector p : trail) {
      // 限制在范围内
      float drawTx = constrain(p.x, xRangeMin, xRangeMax);
      float drawTy = constrain(p.y, yRangeMin, yRangeMax);
      
      float sx = map(drawTx, xRangeMin, xRangeMax, 0, plotWidth);
      float sy = map(drawTy, yRangeMin, yRangeMax, 0, -plotHeight);
      vertex(sx, sy);
    }
  }
  endShape();
  
  // --- 绘制当前数据点 (Set A - 红色) ---
  
  // 检查数据是否在范围内，防止跑到屏幕外
  float drawX = constrain(currentX, xRangeMin, xRangeMax);
  float drawY = constrain(currentY, yRangeMin, yRangeMax);
  
  // 将实际物理坐标 (0-5) 映射到 屏幕像素坐标
  // map(value, start1, stop1, start2, stop2)
  float screenX = map(drawX, xRangeMin, xRangeMax, 0, plotWidth);
  float screenY = map(drawY, yRangeMin, yRangeMax, 0, -plotHeight); // Y轴向上
  
  // 画点 (红色圆形)
  noStroke();
  fill(255, 0, 0);
  ellipse(screenX, screenY, 8, 8); 
  
  // 显示当前坐标值 (Set A - 黑色文字)
  fill(0);
  textAlign(LEFT);
  text("A: (" + nf(currentX, 1, 2) + ", " + nf(currentY, 1, 2) + ")", screenX + 10, screenY - 10);
  
  // --- 绘制当前数据点 (Set B - 绿色) ---
  
  float drawBX = constrain(currentBX, xRangeMin, xRangeMax);
  float drawBY = constrain(currentBY, yRangeMin, yRangeMax);
  
  float screenBX = map(drawBX, xRangeMin, xRangeMax, 0, plotWidth);
  float screenBY = map(drawBY, yRangeMin, yRangeMax, 0, -plotHeight); 
  
  // 画点 (绿色圆形)
  noStroke();
  fill(0, 200, 0);
  ellipse(screenBX, screenBY, 8, 8); 
  
  // 显示当前坐标值 (Set B - 绿色文字)
  fill(0, 150, 0);
  textAlign(LEFT);
  text("B: (" + nf(currentBX, 1, 2) + ", " + nf(currentBY, 1, 2) + ")", screenBX + 10, screenBY + 20);
}

// 串口事件处理
void serialEvent(Serial myPort) {
  try {
    data = myPort.readStringUntil('\n'); // 读取直到换行符
    if (data != null) {
      data = trim(data); // 去除空格和换行
      
      // 按逗号分割字符串: "aX,aY,bX,bY"
      String items[] = split(data, ','); 
      
      if (items.length >= 4) { // 指令变更为4个数据
        // 解析数据
        float inX = float(items[0]); // aX
        float inY = float(items[1]); // aY
        float inBX = float(items[2]); // bX
        float inBY = float(items[3]); // bY
        
        // 只有当数据有效(非NaN)时才更新全局变量
        if (!Float.isNaN(inX) && !Float.isNaN(inY) && !Float.isNaN(inBX) && !Float.isNaN(inBY)) {
          // 更新 Set A
          currentX = inX;
          currentY = inY;
          // 更新 Set B
          currentBX = inBX;
          currentBY = inBY;
          
          // 将新点加入轨迹列表 Set A
          synchronized(trail) {
            trail.add(new PVector(currentX, currentY));
            if (trail.size() > 300) { 
               trail.remove(0);
            }
          }
          
          // 将新点加入轨迹列表 Set B
          synchronized(trailB) {
            trailB.add(new PVector(currentBX, currentBY));
            if (trailB.size() > 300) { 
               trailB.remove(0);
            }
          }
        }
      }
    }
  } catch (Exception e) {
    e.printStackTrace();
  }
}
