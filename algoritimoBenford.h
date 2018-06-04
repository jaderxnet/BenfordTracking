#ifndef __TRACKER_H__
#define __TRACKER_H__

#include "ROIDetector.h"
#include "PanFollower.h"
#include "BayesianSegmenter.h"
#include "DebugRenderer.h"
#include <vector>

class Tracker
{
public:

//Arrais dos valores para cada digito nos 3 tipos de calculo
	  int triploBF[9];
	  int espacialBF[9];
	  int temporalBF[9];
//arrai de avaliação da conformidade com base no chi quadrado
	  double Chi[9];
//Valores de Benford para 9 dígitos
	  float valoresBF[9];
//média final para cada tipo de cálculo
	  float acumuladoTriplo;
	  float acumuladoEspacial;
	  float acumuladoTemporal;
//Variável para a imagem, matriz de pixels
	  cv::Mat img;
//Variável para calcular o vetor gradiente
	  float vertorGrad [240*320][10];
//Ainda não sei o que é o nível ******************************************
	  int nivel;
	  int nivelAnterior;
//Acho qeu são coisas para a tela que vai desenhar o gráfico ******************************************
	  vector<vector<int> > janela;
	  int height;
	  int width;
	  int tamanhoJanela;

//Ainda não sei o que é ******************************************
  Tracker() : succeeded(false)
  {
    cv::Rect currROI = cv::Rect(0, 380, 100, 100);
    detector = new ROIDetector(currROI);
    follower = new PanFollower();
  }


  //Função principal que faz o tratamento dos pixesl
  void track(const cv::Mat frame)
  {
    //DETECT|FOLLOW
    HSVBayesianSegmenter::get()->segment(frame, currMask);
    cv::cvtColor(frame, currGray, CV_BGR2GRAY);

    if(!succeeded) //succeeded was set accordingly to last frame results. if last frame was not successful, then try to detect
    {
      succeeded = detector->detect(currMask, currGray, currFeats, currroi);
    }
    else //otherwise, continue by following
    {
      //succeeded = false;
      succeeded = follower->follow(currMask, lastGray, currGray, lastFeats, currFeats, currP2D, currroi);
      if(!succeeded)
      { //couldnt follow, then try an automatic reinitialization
        cv::Rect nextROI;
        nextROI.x = 2 * currroi.x - lastROI.x;
        nextROI.y = 2 * currroi.y - lastROI.y;
        nextROI.width = currroi.width;
        nextROI.height = currroi.height;
        follower->supplement(currMask, currGray, currFeats, nextROI);
      }
    }

    //REFINE
    //TODO

    //EVALUATE
    //TODO

    //UPDATE
    if(succeeded)
    { //update results, please take care to correctly swap data
      currMask.copyTo(lastMask);
      currGray.copyTo(lastGray);
      lastFeats = currFeats;
      lastROI = currroi;
      lastP2D = currP2D;
    }
    else
    {
      currroi = detector->getInitroi();
    }

    //RENDER
    //rendering debug info
    cv::Mat debugImage(currMask);
    cv::rectangle(debugImage, currroi, CV_RGB(255, 255, 255));
    //cv::add(debugImage, debugImage, debugImage, currMask);
    cv::Scalar color = CV_RGB(200, 200, 200);
    if(succeeded) color = CV_RGB(255, 255, 255);
    for(int i = 0; i < currFeats.size(); ++i) cv::circle(debugImage, currFeats[i].point, 2, color, 1);
    cv::circle(debugImage, currP2D, 5, color, 2);
    int temp = 0;
    if(currFeats.size())
      temp = currFeats[0].point.y;
    string text = "s " + Util::itos(succeeded) + "    f0 " + Util::itos(currFeats.size()) +
                  "   f1 " + Util::itos(lastFeats.size()) + "   fx" + Util::itos(temp) + "       \r";
    printf(text.c_str());
    DebugRenderer::renderImage(debugImage);
  }

private:
  bool succeeded;
  PanFollower* follower;
  ROIDetector* detector;

  //tracker stored data
  Features lastFeats;
  Features currFeats;
  cv::Mat lastMask;
  cv::Mat currMask;
  cv::Mat lastGray;
  cv::Mat currGray;
  cv::Point2f lastP2D;
  cv::Point2f currP2D;
  cv::Rect lastROI;
  cv::Rect currroi;//*/


	//Chamada inicial do Tracker pelo Main
  Tracker(cv::Size frameSize, int tamanhoDaJanela)
  {
	//java Define o tamanho da janela
	//Inicializa as variáveis
	  tamanhoJanela = tamanhoDaJanela;
	  nivel = 0;
	  height = frameSize.height;
	  width = frameSize.width;
	  acumuladoTriplo = 0;
	  acumuladoEspacial = 0;
	  acumuladoTemporal = 0;
	  //java define quantidade de janelas
	  janela.resize(tamanhoJanela);
	  for (int i = 0; i < tamanhoJanela; ++i) {
		  janela[i].resize(height*width);
	  }

	  //java zerou para inicializar os arrays da distribui��o
	  //com base nos 4 tipos de c�lculos
	  //Gradiente em rela��o ao RGB
	  for (int i = 0; i < 9; i++) {
		  triploBF[i] = 0.0;
	  }
	  //Gradiente em rela��o a XYZ
	  for (int i = 0; i < 9; i++) {
		  espacialBF[i] = 0.0;
	  }
	  // Gradiente em rela��o ao Tempo
	  for (int i = 0; i < 9; i++) {
		  temporalBF[i] = 0.0;
	  }
	  //java calcula a conformidade
	  for (int i = 0; i < 9; i++) {
		  Chi[i] = 0;
	  }
	  //java Inicializa os valores padr�es de Benford
	  valoresBF[0] = log10f(1.0+1.0/1.0);
	  valoresBF[1] = log10f(1.0+1.0/2.0);
	  valoresBF[2] = log10f(1.0+1.0/3.0);
	  valoresBF[3] = log10f(1.0+1.0/4.0);
	  valoresBF[4] = log10f(1.0+1.0/5.0);
	  valoresBF[5] = log10f(1.0+1.0/6.0);
	  valoresBF[6] = log10f(1.0+1.0/7.0);
	  valoresBF[7] = log10f(1.0+1.0/8.0);
	  valoresBF[8] = log10f(1.0+1.0/9.0);
	  //java Inicializou uma matriz
	  this->img = cv::Mat(20, 1000, CV_8UC3, cv::Scalar::all(0));
	  // Cria um retangulo
	  initROI = cv::Rect(0, frameSize.height * 0.4, frameSize.width * 0.15, frameSize.height * 0.2);
	  currROI = initROI;
	  lastROI = currROI;

	  succeeded  = false;
	  memset (vertorGrad, 0, sizeof(vertorGrad));
  }

  // Implementa uma fila circular para os frames serem usados
  void inserirNovoGradienteFrame(cv::Mat& image) {
	  nivelAnterior = nivel;

	  for (int i = 0; i < height*width; ++i) {
		  janela[nivel][i] = image.data[i];
	  }
	  printf("%d \r",  nivel);
	  //atualizar o nivel das janela
	  nivel = (nivel+1)%tamanhoJanela;
  }

  //Calcula Gradiente triplo
  void gradienteTriplo10(cv::Mat& image) {
	  inserirNovoGradienteFrame(image);
	  int countLevel = 0;
	  float normaTripla = 0.0,
		    normaEspacial = 0.0,
			normaTemporal = 0.0;

	  float gradiente[3] = {0.0, 0.0, 0.0};

	  for(int x = 0; x < 9; ++x) {
		  triploBF[x] = 0;
		  espacialBF[x] = 0;
		  temporalBF[x] = 0;
	  }//printf("%.10f \r",  acumuladoTriplo);

	  //acumulador de ocorencia de 1..9
	  for (int n = 1; n < tamanhoJanela; ++n) {
		  for (int index = 1; index < (height*width)-width; ++index) {
			  gradiente[0] = janela[n][index+1] - janela[n][index];
			  gradiente[1] = janela[n][index+image.cols] - janela[n][index];
			  gradiente[2] = janela[n][index] - janela[n-1][index];
			  normaTripla = sqrt(gradiente[0]*gradiente[0] + gradiente[1]*gradiente[1] + gradiente[2]*gradiente[2]);
			  normaEspacial = sqrt(gradiente[0]*gradiente[0] + gradiente[1]*gradiente[1]);
			  normaTemporal = gradiente[2];

			  if (normaTripla > 0.001) {//se a norma for significativa
				  if(normaTripla > 100) {
					  countLevel = normaTripla/100;
					  triploBF[countLevel-1]++;
				  } else if (normaTripla > 10) {
					  countLevel = normaTripla/10;
					  triploBF[countLevel-1]++;
				  } else {
					  countLevel = normaTripla;
					  triploBF[countLevel-1]++;
				  }
			  }

			  if (normaEspacial > 0.001) {//se a norma espacial for significativa
				  if(normaEspacial > 100) {
					  countLevel = normaEspacial/100;
					  espacialBF[countLevel-1]++;
				  } else if (normaEspacial > 10) {
					  countLevel = normaEspacial/10;
					  espacialBF[countLevel-1]++;
				  } else {
					  countLevel = normaEspacial;
					  espacialBF[countLevel-1]++;
				  }
			  }

			  if (normaTemporal > 0.001) {//se a norma temporal for significativa
				  if(normaTemporal > 100) {
					  countLevel = normaTemporal/100;
					  temporalBF[countLevel-1]++;
				  } else if (normaTemporal > 10) {
					  countLevel = normaTemporal/10;
					  temporalBF[countLevel-1]++;
				  } else {
					  countLevel = normaTemporal;
					  temporalBF[countLevel-1]++;
				  }
			  }

		  }
	  }
	  //calculo o total de valores
	  acumuladoTriplo = 0;
	  acumuladoEspacial = 0;
	  acumuladoTemporal = 0;
	  for (int i = 0; i < 9; i++) {
		  acumuladoTriplo += triploBF[i];
		  acumuladoEspacial += espacialBF[i];
		  acumuladoTemporal += temporalBF[i];
	  }

	  imagemConformidade();
  }

  void drawRec () {
	//VERDE AMARELO VERMELHO
	  float diferencaTripla = 0.0;
	  float diferencaEspacial = 0.0;
	  float diferencaTemporal = 0.0;

	  for(int nivel = 0; nivel < 9; nivel++) {
		  if (((triploBF[nivel]/acumuladoTriplo) - valoresBF[nivel]) > 0) {
			diferencaTripla += (triploBF[nivel]/acumuladoTriplo) - valoresBF[nivel];
		  } else {
			diferencaTripla -= (triploBF[nivel]/acumuladoTriplo) - valoresBF[nivel];
		  }
	  }
	  diferencaTripla /= 2;
	  //printf ("%.3f     \r", dif);


	  for(int nivel = 0; nivel < 9; nivel++) {
		  if (((espacialBF[nivel]/acumuladoEspacial) - valoresBF[nivel]) > 0) {
			diferencaEspacial += (espacialBF[nivel]/acumuladoEspacial) - valoresBF[nivel];
		  } else {
			diferencaEspacial -= (espacialBF[nivel]/acumuladoEspacial) - valoresBF[nivel];
		  }
	  }
	  diferencaEspacial /= 2;
	  //printf ("%.3f     \r", dif);

	  for(int nivel = 0; nivel < 9; nivel++) {
		  if (((temporalBF[nivel]/acumuladoTemporal) - valoresBF[nivel]) > 0) {
			diferencaTemporal += (temporalBF[nivel]/acumuladoTemporal) - valoresBF[nivel];
		  } else {
			diferencaTemporal -= (temporalBF[nivel]/acumuladoTemporal) - valoresBF[nivel];
		  }
	  }
	  diferencaTemporal /= 2;
	  //printf ("%.3f     \r", dif);

	  //FILE * fid = fopen ("gradiente.txt","a");
	  //fprintf(fid, "%.10f \r\n",  dif );
      //fclose(fid);
	  cv::Rect conformidadeTripla(0, 0, 20, 300*(1 - diferencaTripla));
	  cv::rectangle(this->img, conformidadeTripla, cv::Scalar(255, 255*(1 - diferencaTripla), 255*diferencaTripla), CV_FILLED);

	  cv::Rect conformidadeEspacial(20, 0, 20, 300*(1 - diferencaEspacial));
	  cv::rectangle(this->img, conformidadeEspacial, cv::Scalar(125, 255*(1 - diferencaEspacial), 255*diferencaEspacial), CV_FILLED);

	  cv::Rect conformidadeTemporal(40, 0, 20, 300*(1 - diferencaTemporal));
	  cv::rectangle(this->img, conformidadeTemporal, cv::Scalar(0, 255*(1 - diferencaTemporal), 255*diferencaTemporal), CV_FILLED);



	  cv::Rect barra5(0, 15, 20, 2);
	  cv::rectangle(this->img, barra5, cv::Scalar(125, 0, 0), CV_FILLED);

	  cv::Rect barra10(0, 30, 40, 2);
	  cv::rectangle(this->img, barra10, cv::Scalar(255, 0, 0), CV_FILLED);

	  cv::Rect barra15(0, 45, 20, 2);
	  cv::rectangle(this->img, barra15, cv::Scalar(125, 0, 0), CV_FILLED);

	  cv::Rect barra20(0, 60, 40, 2);
	  cv::rectangle(this->img, barra20, cv::Scalar(255, 0, 0), CV_FILLED);

	  cv::Rect barra25(0, 75, 20, 2);
	  cv::rectangle(this->img, barra25, cv::Scalar(125, 0, 0), CV_FILLED);

	  cv::Rect barra30(0, 90, 40, 2);
	  cv::rectangle(this->img, barra30, cv::Scalar(255, 0, 0), CV_FILLED);

	  cv::Rect barra35(0, 105, 20, 2);
	  cv::rectangle(this->img, barra35, cv::Scalar(125, 0, 0), CV_FILLED);

	  cv::Rect barra40(0, 120, 40, 2);
	  cv::rectangle(this->img, barra40, cv::Scalar(255, 0, 0), CV_FILLED);

	  cv::Rect barra45(0, 135, 20, 2);
	  cv::rectangle(this->img, barra45, cv::Scalar(125, 0, 0), CV_FILLED);

	  cv::Rect barra50(0, 150, 40, 2);
	  cv::rectangle(this->img, barra50, cv::Scalar(255, 0, 0), CV_FILLED);

	  cv::Rect barra55(0, 165, 20, 2);
	  cv::rectangle(this->img, barra55, cv::Scalar(125, 0, 0), CV_FILLED);

	  cv::Rect barra60(0, 180, 40, 2);
	  cv::rectangle(this->img, barra60, cv::Scalar(255, 0, 0), CV_FILLED);

	  cv::Rect barra65(0, 195, 20, 2);
	  cv::rectangle(this->img, barra65, cv::Scalar(125, 0, 0), CV_FILLED);

	  cv::Rect barra70(0, 210, 40, 2);
	  cv::rectangle(this->img, barra70, cv::Scalar(255, 0, 0), CV_FILLED);

	  cv::Rect barra75(0, 225, 20, 2);
	  cv::rectangle(this->img, barra75, cv::Scalar(125, 0, 0), CV_FILLED);

	  cv::Rect barra80(0, 240, 40, 2);
	  cv::rectangle(this->img, barra80, cv::Scalar(255, 0, 0), CV_FILLED);

	  cv::Rect barra85(0, 255, 20, 2);
	  cv::rectangle(this->img, barra85, cv::Scalar(125, 0, 0), CV_FILLED);

	  cv::Rect barra90(0, 270, 40, 2);
	  cv::rectangle(this->img, barra90, cv::Scalar(255, 0, 0), CV_FILLED);

	  cv::Rect barra95(0, 285, 20, 2);
	  cv::rectangle(this->img, barra95, cv::Scalar(125, 0, 0), CV_FILLED);

	/*float dif;
	for (int nivel = 0; nivel < 9; nivel ++) {
		dif = (BF[nivel]/acumuladoTriplo) - valoresBF[nivel];
		if (dif > 0) {
			cv::Rect primeiroDigitoBF(150 + nivel*100, 0, 16, 10);
			cv::rectangle(this->img, primeiroDigitoBF, cv::Scalar(0, 255000*dif, 0), CV_FILLED);
		} else {
			cv::Rect primeiroDigitoBF(150 + nivel*100, 0, 16, 10);
			cv::rectangle(this->img, primeiroDigitoBF, cv::Scalar(-255000*dif, 0, 0), CV_FILLED);
		}
	}*/
  }

  void imagemConformidade (){
	  grafico();
	  drawRec();
	  DebugRenderer::renderImage(this->img);
  }

  void grafico (){
	// a ser otimizado
	img = cv::Mat(300, 1000, CV_8UC3, cv::Scalar::all(0));
	//base
	cv::Rect primeiroDigito(975, 0, 25, 500*valoresBF[0]);
	cv::rectangle(img, primeiroDigito, cv::Scalar(0, 0, 255), CV_FILLED);
	cv::Rect segundoDigito(875, 0, 25, 500*valoresBF[1]);
	cv::rectangle(img, segundoDigito, cv::Scalar(0, 0, 255), CV_FILLED);
	cv::Rect terceiroDigito(775, 0, 25, 500*valoresBF[2]);
	cv::rectangle(img, terceiroDigito, cv::Scalar(0, 0, 255), CV_FILLED);
	cv::Rect quartoDigito(675, 0, 25, 500*valoresBF[3]);
	cv::rectangle(img, quartoDigito, cv::Scalar(0, 0, 255), CV_FILLED);
	cv::Rect quintoDigito(575, 0, 25, 500*valoresBF[4]);
	cv::rectangle(img, quintoDigito, cv::Scalar(0, 0, 255), CV_FILLED);
	cv::Rect sextoDigito(475, 0, 25, 500*valoresBF[5]);
	cv::rectangle(img, sextoDigito, cv::Scalar(0, 0, 255), CV_FILLED);
	cv::Rect setimoDigito(375, 0, 25, 500*valoresBF[6]);
	cv::rectangle(img, setimoDigito, cv::Scalar(0, 0, 255), CV_FILLED);
	cv::Rect oitavoDigito(275, 0, 25, 500*valoresBF[7]);
	cv::rectangle(img, oitavoDigito, cv::Scalar(0, 0, 255), CV_FILLED);
	cv::Rect nonoDigito(175, 0, 25, 500*valoresBF[8]);
	cv::rectangle(img, nonoDigito, cv::Scalar(0, 0, 255), CV_FILLED);
	//base

	//triplo
	cv::Rect primeiroDigitoTriplo(950, 0, 25, 500*(triploBF[0]/acumuladoTriplo));
	cv::rectangle(img, primeiroDigitoTriplo, cv::Scalar(255, 128, 0), CV_FILLED);
	cv::Rect segundoDigitoTriplo(850, 0, 25, 500*(triploBF[1]/acumuladoTriplo));
	cv::rectangle(img, segundoDigitoTriplo, cv::Scalar(255, 128, 0), CV_FILLED);
	cv::Rect terceiroDigitoTriplo(750, 0, 25, 500*(triploBF[2]/acumuladoTriplo));
	cv::rectangle(img, terceiroDigitoTriplo, cv::Scalar(255, 128, 0), CV_FILLED);
	cv::Rect quartoDigitoTriplo(650, 0, 25, 500*(triploBF[3]/acumuladoTriplo));
	cv::rectangle(img, quartoDigitoTriplo, cv::Scalar(255, 128, 0), CV_FILLED);
	cv::Rect quintoDigitoTriplo(550, 0, 25, 500*(triploBF[4]/acumuladoTriplo));
	cv::rectangle(img, quintoDigitoTriplo, cv::Scalar(255, 128, 0), CV_FILLED);
	cv::Rect sextoDigitoTriplo(450, 0, 25, 500*(triploBF[5]/acumuladoTriplo));
	cv::rectangle(img, sextoDigitoTriplo, cv::Scalar(255, 128, 0), CV_FILLED);
	cv::Rect setimoDigitoTriplo(350, 0, 25, 500*(triploBF[6]/acumuladoTriplo));
	cv::rectangle(img, setimoDigitoTriplo, cv::Scalar(255, 128, 0), CV_FILLED);
	cv::Rect oitavoDigitoTriplo(250, 0, 25, 500*(triploBF[7]/acumuladoTriplo));
	cv::rectangle(img, oitavoDigitoTriplo, cv::Scalar(255, 128, 0), CV_FILLED);
	cv::Rect nonoDigitoTriplo(150, 0, 25, 500*(triploBF[8]/acumuladoTriplo));
	cv::rectangle(img, nonoDigitoTriplo, cv::Scalar(255, 128, 0), CV_FILLED);
	//triplo

	//espacial
	cv::Rect primeiroDigitoEspacial(925, 0, 25, 500*(espacialBF[0]/acumuladoTriplo));
	cv::rectangle(img, primeiroDigitoEspacial, cv::Scalar(255, 128, 220), CV_FILLED);
	cv::Rect segundoDigitoEspacial(825, 0, 25, 500*(espacialBF[1]/acumuladoTriplo));
	cv::rectangle(img, segundoDigitoEspacial, cv::Scalar(255, 128, 220), CV_FILLED);
	cv::Rect terceiroDigitoEspacial(725, 0, 25, 500*(espacialBF[2]/acumuladoTriplo));
	cv::rectangle(img, terceiroDigitoEspacial, cv::Scalar(255, 128, 220), CV_FILLED);
	cv::Rect quartoDigitoEspacial(625, 0, 25, 500*(espacialBF[3]/acumuladoTriplo));
	cv::rectangle(img, quartoDigitoEspacial, cv::Scalar(255, 128, 220), CV_FILLED);
	cv::Rect quintoDigitoEspacial(525, 0, 25, 500*(espacialBF[4]/acumuladoTriplo));
	cv::rectangle(img, quintoDigitoEspacial, cv::Scalar(255, 128, 220), CV_FILLED);
	cv::Rect sextoDigitoEspacial(425, 0, 25, 500*(espacialBF[5]/acumuladoTriplo));
	cv::rectangle(img, sextoDigitoEspacial, cv::Scalar(255, 128, 220), CV_FILLED);
	cv::Rect setimoDigitoEspacial(325, 0, 25, 500*(espacialBF[6]/acumuladoTriplo));
	cv::rectangle(img, setimoDigitoEspacial, cv::Scalar(255, 128, 220), CV_FILLED);
	cv::Rect oitavoDigitoEspacial(225, 0, 25, 500*(espacialBF[7]/acumuladoTriplo));
	cv::rectangle(img, oitavoDigitoEspacial, cv::Scalar(255, 128, 220), CV_FILLED);
	cv::Rect nonoDigitoEspacial(125, 0, 25, 500*(espacialBF[8]/acumuladoTriplo));
	cv::rectangle(img, nonoDigitoEspacial, cv::Scalar(255, 128, 220), CV_FILLED);
	//espacial

	//temporal
	cv::Rect primeiroDigitoTemporal(900, 0, 25, 500*(temporalBF[0]/acumuladoTemporal));
	cv::rectangle(img, primeiroDigitoTemporal, cv::Scalar(255, 255, 255), CV_FILLED);
	cv::Rect segundoDigitoTemporal(800, 0, 25, 500*(temporalBF[1]/acumuladoTemporal));
	cv::rectangle(img, segundoDigitoTemporal, cv::Scalar(255, 255, 255), CV_FILLED);
	cv::Rect terceiroDigitoTemporal(700, 0, 25, 500*(temporalBF[2]/acumuladoTemporal));
	cv::rectangle(img, terceiroDigitoTemporal, cv::Scalar(255, 255, 255), CV_FILLED);
	cv::Rect quartoDigitoTemporal(600, 0, 25, 500*(temporalBF[3]/acumuladoTemporal));
	cv::rectangle(img, quartoDigitoTemporal, cv::Scalar(255, 255, 255), CV_FILLED);
	cv::Rect quintoDigitoTemporal(500, 0, 25, 500*(temporalBF[4]/acumuladoTemporal));
	cv::rectangle(img, quintoDigitoTemporal, cv::Scalar(255, 255, 255), CV_FILLED);
	cv::Rect sextoDigitoTemporal(400, 0, 25, 500*(temporalBF[5]/acumuladoTemporal));
	cv::rectangle(img, sextoDigitoTemporal, cv::Scalar(255, 255, 255), CV_FILLED);
	cv::Rect setimoDigitoTemporal(300, 0, 25, 500*(temporalBF[6]/acumuladoTemporal));
	cv::rectangle(img, setimoDigitoTemporal, cv::Scalar(255, 255, 255), CV_FILLED);
	cv::Rect oitavoDigitoTemporal(200, 0, 25, 500*(temporalBF[7]/acumuladoTemporal));
	cv::rectangle(img, oitavoDigitoTemporal, cv::Scalar(255, 255, 255), CV_FILLED);
	cv::Rect nonoDigitoTemporal(100, 0, 25, 500*(temporalBF[8]/acumuladoTemporal));
	cv::rectangle(img, nonoDigitoTemporal, cv::Scalar(255, 255, 255), CV_FILLED);
	//temporal


	cv::Point2f centro(img.cols/2.0F, img.rows/2.0F);
	cv::Mat dst;
	cv::warpAffine(img, dst, cv::getRotationMatrix2D(centro, 180, 1.0), img.size());
	DebugRenderer::renderImage(dst);
  }

  cv::Mat rotateImage(const cv::Mat& source, double angle)
{
    cv::Point2f src_center(source.cols/2.0F, source.rows/2.0F);
    cv::Mat rot_mat = getRotationMatrix2D(src_center, angle, 1.0);
    cv::Mat dst;
    warpAffine(source, dst, rot_mat, source.size());
    return dst;
}


  void track(cv::Mat frame)
  {
    //HSVBayesianSegmenter::get()->segment(frame, currMask);
    cv::cvtColor(frame, currGray, CV_BGR2GRAY);
    //labeling(currMask);

	gradienteTriplo10(currGray);

      //succeeded is initialized as false
    /*if(!succeeded)
    {
      succeeded = ROIDetector::get()->detect(currGray, currMask, initROI, currFeatures);// pointsCount, points[1]);
    }
    else
    {
      succeeded = PanFollower::get()->follow(currGray, lastGray, currMask, currROI, lastROI, currFeatures, lastFeatures, medianPoint);

      if(!succeeded)
      { //feature auto reinitialization
        PanFollower::get()->complement(currGray, lastROI, currMask, currFeatures);
        PanFollower::get()->complement(currGray, currROI, currMask, currFeatures);
        //PanFollower::get()->supplement(nextROI, currMask, currGray, currFeatures);
        if(currFeatures.size() > 0)
        {
          PanFollower::get()->calculateMedianPoint(currFeatures, medianPoint);
          Util::commonBox(medianPoint, frame.size(), PanFollower::get()->roiRelativeIncrement, currROI);

          cv::rectangle(frame, lastROI, CV_RGB(64,  64,  64));
          cv::rectangle(frame, currROI, CV_RGB(200, 200, 200));
          //cv::rectangle(frame, nextROI, CV_RGB(255, 255, 255));
          succeeded = true;
        }
      }
    }
    cv::rectangle(frame, currROI, CV_RGB(0, 200, 0));

    cv::swap(currGray, lastGray);
    lastFeatures = currFeatures;*/
    //CV_SWAP(lastGray, currGray, imageSwap);
    //CV_SWAP(points[0], points[1], pointsSwap);
  }


private:
	cv::Rect initROI;
    cv::Rect currROI;
    cv::Rect lastROI;
    //cv::Rect nextROI;

    //cv::Mat imageSwap;
    cv::Mat currMask;
    cv::Mat currGray;
    cv::Mat lastGray;

    cv::Point2f medianPoint;

    Features currFeatures;
    Features lastFeatures;
    //CvPoint2D32f* pointsMissing;
    //CvPoint2D32f* points[2];
    //CvPoint2D32f* pointsSwap;

    //int pointsCount;

    bool succeeded;
};

#endif //__TRACKER_H__
