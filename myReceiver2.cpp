
#include <uhd/utils/thread_priority.hpp>
#include <uhd/utils/safe_main.hpp>
#include <uhd/usrp/multi_usrp.hpp>
#include <uhd/transport/udp_simple.hpp>
#include <uhd/exception.hpp>
#include <boost/program_options.hpp>
#include <boost/format.hpp>
#include <boost/thread.hpp>
#include <boost/lexical_cast.hpp>
#include <iostream>
#include <complex>
#include <csignal>
#include <iomanip>
#include <fftw3.h>
#include <numeric>
#include <curses.h>
#include <iterator>
#include <string>
#include <vector>

void print_data ( std::vector<std::complex<float> > out_buff)
{
  for (unsigned int i=0; i<out_buff.size();i++)
    std::cout<<out_buff[i]<<std::endl;
       
}

float find_energy (std::vector<std::complex<float> > out_buff)

{
  float   energy =0;

  for (unsigned int i=0;i<out_buff.size();i++)
    energy = pow( std::abs(out_buff[i]),2)+energy;



  return energy;
}


namespace po =boost::program_options;
static bool stop_signal_called = false;
void sig_int_handler(int){stop_signal_called = true;}



//set program_options
int UHD_SAFE_MAIN (int argc, char *argv[])
{ 
   uhd::set_thread_priority_safe();
   std::string args;
   size_t total_samples,number_bins,num_acc_samps;
   double bw,rate, freq, gain;
   po::options_description desc("allowed options");

   desc.add_options()
	("args",po::value<std::string>(&args)->default_value(""),"multi uhd device address args")
   ("help","help message")
  ("nsamps",po::value<size_t> (&total_samples)->default_value(0),"Total number of samples to receive, zero for continous mode")
     ("rate", po::value<double>(&rate)->default_value(2e6), "rate of incoming samples")
   ("freq",po::value<double>(&freq)->default_value(400e6),"rf center frequency in Hz")
   ("gain",po::value<double>(&gain)->default_value(0),"gain for the RF chain")
   ("number_bins",po::value<size_t>(&number_bins)->default_value(1024),"number of FFT points")
     ("bw", po::value<double>(&bw), "daughterboard IF filter bandwidth in Hz")

   ;
 po::variables_map vm;
 po::store(po::parse_command_line(argc,argv,desc),vm);
 po::notify(vm);


 if (vm.count("help"))
 	{//if
 		std::cout<< boost::format("UHD RX timed Samples %s") % desc <<std::endl;
		return ~0; 
	}//if

//create usrp device
	std::cout<<std::endl;
	std::cout<<boost::format("setting RX Rate: %f Msps...") % args <<std::endl;
	uhd::usrp::multi_usrp::sptr usrp =uhd::usrp::multi_usrp::make(args);
	std::cout<<boost::format("Using Device: %s ") % usrp->get_pp_string()<<std::endl;
	//set bandwidth
	if (vm.count("bw")){
	  std::cout << boost::format("Setting RX Bandwidth: %f MHz...") % bw << std::endl;
	  usrp->set_rx_bandwidth(bw);
	  std::cout << boost::format("Actual RX Bandwidth: %f MHz...") % usrp->get_rx_bandwidth() << std::endl << std::endl;
	}
//set the sample rate

	std::cout << boost::format("setting RX Rate: %f Msps...") % (rate/1e6) <<std::endl<<std::endl;
	usrp->set_rx_rate(rate);
	std::cout<<boost::format("actual RX rate: %f Msps...") % (usrp->get_rx_rate()/1e6) <<std::endl<<std::endl;

//set the rx center frequency
    std::cout << boost::format("Setting RX Freq: %f Mhz...") % (freq/1e6) << std::endl;
    usrp->set_rx_freq(freq);
    std::cout << boost::format("Actual RX Freq: %f Mhz...") % (usrp->get_rx_freq()/1e6) << std::endl << std::endl;


//create a receiver streamer
    uhd::stream_args_t stream_args("fc32");
    uhd::rx_streamer::sptr rx_streamer =usrp-> get_rx_stream(stream_args);

//rm// set up streaming ...0 means continues
    uhd::stream_cmd_t stream_cmd((total_samples==0)?
    uhd::stream_cmd_t::STREAM_MODE_START_CONTINUOUS:
    uhd::stream_cmd_t::STREAM_MODE_NUM_SAMPS_AND_DONE);



stream_cmd.num_samps =total_samples;
stream_cmd.stream_now = true;
stream_cmd.time_spec =uhd::time_spec_t();
usrp->issue_stream_cmd(stream_cmd);

size_t num_rx_samps =0; //initialize number of received samples

uhd::rx_metadata_t md;

std::vector<std::complex<float> > buff(number_bins);
std::vector<std::complex<float> > out_buff(number_bins);

//initialize fft plan

fftwf_complex *in = (fftwf_complex*)&buff.front(); //allocate  array in
fftwf_complex *out = (fftwf_complex*)&out_buff.front(); //allocate array out
fftwf_plan f;  
f =fftwf_plan_dft_1d(number_bins,in, out, FFTW_FORWARD,FFTW_ESTIMATE);

 while(not stop_signal_called and (num_acc_samps < total_samples or total_samples == 0))
   {
     size_t num_rx_samps = rx_streamer->recv( &buff.front(), buff.size(), md, 3.0);
     std::cout <<" current buffer size: "<< buff.size()<<std::endl<<std::endl;   
     //handle the error codes
     switch(md.error_code){
     case uhd::rx_metadata_t::ERROR_CODE_NONE:
       break;

     case uhd::rx_metadata_t::ERROR_CODE_TIMEOUT:
       if (num_acc_samps == 0) continue;
       std::cout << boost::format(
                "Got timeout before all samples received, possible packet loss, exiting loop..."
				  ) << std::endl;
       goto done_loop;

     default:
       std::cout << boost::format(
                "Got error code 0x%x, exiting loop..."
				  ) % md.error_code << std::endl;
       goto done_loop;
     }

     std::cout<<"performing fft to samples at frequency"<<usrp->get_rx_freq()<<std::endl;
     fftwf_execute(f);
     num_acc_samps = num_rx_samps +1;

     std::cout<<"number of accumulated samples"<<num_acc_samps<<std::endl<<std::endl;
     std::cout <<"nubmer of rx samples: "<<num_rx_samps <<std::endl<<std::endl;
     float energy = find_energy(out_buff);
     std::cout<<"the energy for incoming samples: " <<energy;

     //     print_data(out_buff);  
     
   }


done_loop:

fftwf_destroy_plan(f);

std::cout<<std::endl<<"done";
return 0;

 }




